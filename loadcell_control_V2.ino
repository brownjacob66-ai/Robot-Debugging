// =============================================================================
// FORCE UNO ARDUINO SKETCH (Load Cell Sensor Interface)
// =============================================================================
//
// Purpose:
//   - Read real-time force from load cell via HX711 amplifier
//   - Convert raw ADC values to grams with calibration
//   - Send force readings via serial at high frequency
//   - Provide debouncing and filtering for stable readings
//   - Generate hardware E-Stop signal on probe contact detection
//   - Implement state machine to prevent false triggers
//
// Hardware Connections:
//   HX711 DT (Data)  → Arduino Pin 3
//   HX711 SCK (Clock) → Arduino Pin 2
//   E-Stop Output    → Arduino Pin 8 (to Motion Mega)
//   HX711 GND        → Arduino GND
//   HX711 VCC        → Arduino 5V
//
// Load Cell Wiring (4-wire configuration):
//   Red (Excitation +)   → HX711 E+
//   Black (Excitation -) → HX711 E-
//   White (Signal +)     → HX711 A+
//   Green (Signal -)     → HX711 A-
//
// Serial Output Protocol:
//   "FORCE:XXX.X\n"  (e.g., "FORCE:152.3\n")
//   Sent at ~67 Hz (every 15ms)
//
// Hardware E-Stop Signal:
//   Pin 8 goes HIGH when probe contact detected (40g threshold)
//   Held HIGH for 100ms to ensure Motion Mega catches it
//   Then drops LOW
//   5-second lockout prevents re-triggering during single probe
//
// =============================================================================

#include "HX711.h"

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

const int LOADCELL_DOUT_PIN = 3;   // HX711 Data output
const int LOADCELL_SCK_PIN = 2;    // HX711 Clock
const int ESTOP_OUT_PIN = 8;       // Hardware E-Stop signal to Motion Mega

// =============================================================================
// CALIBRATION & SENSOR CONFIGURATION
// =============================================================================

// Calibration factor: HX711 units per gram
// This is YOUR existing calibration value
// If load cell readings are inverted, negate this value
const float CALIBRATION_FACTOR = -25.19;

// Probe contact detection threshold (grams)
// When smoothed force exceeds this, trigger E-Stop
const float PROBE_CONTACT_THRESHOLD = 40.0;

// Probe lockout: exit when force drops below this fraction of threshold
// Prevents re-trigger while dop is still in contact
const float PROBE_RELEASE_THRESHOLD = PROBE_CONTACT_THRESHOLD * 0.5;  // 20.0g

// Serial reporting interval (milliseconds)
// 15ms = ~67 Hz update rate
const unsigned long REPORT_INTERVAL_MS = 15;

// =============================================================================
// MOVING AVERAGE FILTER
// =============================================================================

// Circular buffer for force readings
const int FILTER_SIZE = 12;
float force_readings[FILTER_SIZE];
int filter_index = 0;
float filter_total = 0.0;

// =============================================================================
// STATE MACHINE FOR PROBE DETECTION
// =============================================================================

enum ProbeState {
  STATE_IDLE,       // Waiting for probe contact
  STATE_TRIGGERED,  // Contact detected, holding E-Stop signal HIGH
  STATE_LOCKOUT     // Preventing re-trigger while force still applied
};

ProbeState probe_state = STATE_IDLE;
unsigned long state_timer = 0;

// Timing constants
const unsigned long ESTOP_HOLD_TIME_MS = 100;    // How long to hold HIGH
const unsigned long LOCKOUT_TIME_MS = 5000;      // 5 second lockout

// =============================================================================
// HX711 LOAD CELL OBJECT
// =============================================================================

HX711 scale;

// =============================================================================
// SETUP PROCEDURE
// =============================================================================

void setup() {
  // Initialize serial communication at high baud rate
  Serial.begin(115200);
  
  // Configure E-Stop output pin
  pinMode(ESTOP_OUT_PIN, OUTPUT);
  digitalWrite(ESTOP_OUT_PIN, LOW);  // Start in LOW state (inactive)
  
  // Initialize HX711 load cell amplifier
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // Wait for scale to be ready
  delay(1000);
  
  // Set calibration factor (converts ADC units to grams)
  scale.set_scale(CALIBRATION_FACTOR);
  
  // Tare (zero) the scale
  // Ensure load cell is empty when powering on
  scale.tare();
  
  // Initialize filter array with zeros
  for (int i = 0; i < FILTER_SIZE; i++) {
    force_readings[i] = 0.0;
  }
  filter_total = 0.0;
  filter_index = 0;
  
  // Ready signal
  Serial.println("FORCE_MEGA_READY");
  delay(100);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

unsigned long last_report_time = 0;

void loop() {
  // Check if scale is ready (HX711 has data available)
  if (scale.is_ready()) {
    // Read raw force value in grams
    float raw_force = scale.get_units();
    
    // Apply moving average filter for smooth readings
    float filtered_force = applyMovingAverage(raw_force);
    
    // Execute probe state machine
    updateProbeStateMachine(filtered_force);
    
    // Send force reading to host (non-blocking, interval-based)
    unsigned long current_time = millis();
    if (current_time - last_report_time >= REPORT_INTERVAL_MS) {
      reportForce(filtered_force);
      last_report_time = current_time;
    }
  }
  
  // Small delay to prevent CPU overload
  delayMicroseconds(100);
}

// =============================================================================
// MOVING AVERAGE FILTER FUNCTION
// =============================================================================

float applyMovingAverage(float new_reading) {
  /*
   * Circular buffer moving average:
   * - Store most recent FILTER_SIZE readings
   * - Always maintain sum of all readings
   * - Calculate average on each call
   * - O(1) time complexity (no loop needed)
   */
  
  // Remove oldest reading from sum
  filter_total -= force_readings[filter_index];
  
  // Add new reading to buffer and sum
  force_readings[filter_index] = new_reading;
  filter_total += new_reading;
  
  // Move to next buffer position (wraps around)
  filter_index = (filter_index + 1) % FILTER_SIZE;
  
  // Calculate and return average
  float average = filter_total / (float)FILTER_SIZE;
  
  // Clamp to zero (no negative force)
  if (average < 0.0) {
    average = 0.0;
  }
  
  return average;
}

// =============================================================================
// PROBE STATE MACHINE
// =============================================================================

void updateProbeStateMachine(float filtered_force) {
  /*
   * State Machine Flow:
   * 
   * STATE_IDLE (default):
   *   - Monitor force for threshold crossing
   *   - If force >= threshold: set pin HIGH, go to TRIGGERED
   * 
   * STATE_TRIGGERED (probe contact detected):
   *   - Hold E-Stop pin HIGH for 100ms
   *   - Ensures Motion Mega has time to read the signal
   *   - After 100ms: drop pin LOW, go to LOCKOUT
   * 
   * STATE_LOCKOUT (debounce protection):
   *   - Prevent re-triggering for 5 seconds
   *   - OR exit early if force drops below 50% of threshold
   *   - Prevents multiple triggers during single probe operation
   *   - Return to IDLE when either condition met
   */
  
  switch (probe_state) {
    
    case STATE_IDLE:
      // Waiting for probe contact
      if (filtered_force >= PROBE_CONTACT_THRESHOLD) {
        // Threshold exceeded - contact detected
        digitalWrite(ESTOP_OUT_PIN, HIGH);
        probe_state = STATE_TRIGGERED;
        state_timer = millis();
        
        // Optional: could send debug message
        // Serial.println("DEBUG: PROBE_TRIGGERED");
      }
      break;
    
    case STATE_TRIGGERED:
      // Holding E-Stop signal HIGH
      if (millis() - state_timer >= ESTOP_HOLD_TIME_MS) {
        // 100ms elapsed - release the signal
        digitalWrite(ESTOP_OUT_PIN, LOW);
        probe_state = STATE_LOCKOUT;
        state_timer = millis();
        
        // Optional: debug message
        // Serial.println("DEBUG: ENTERING_LOCKOUT");
      }
      break;
    
    case STATE_LOCKOUT:
      // Debounce lockout to prevent false re-triggers
      unsigned long elapsed = millis() - state_timer;
      bool timeout_expired = elapsed >= LOCKOUT_TIME_MS;
      bool force_released = filtered_force < PROBE_RELEASE_THRESHOLD;
      
      if (timeout_expired || force_released) {
        // Exit lockout and return to idle
        probe_state = STATE_IDLE;
        
        // Optional: debug message
        // Serial.print("DEBUG: EXITING_LOCKOUT (");
        // Serial.print(timeout_expired ? "timeout" : "force_released");
        // Serial.println(")");
      }
      break;
  }
}

// =============================================================================
// FORCE REPORTING
// =============================================================================

void reportForce(float force_grams) {
  /*
   * Send force reading to host in standardized format
   * Format: "FORCE:XXX.X\n"
   * Precision: 1 decimal place
   * 
   * This is read by the Python control system at regular intervals
   * High frequency (67Hz) provides responsive feedback
   */
  
  Serial.print("FORCE:");
  Serial.println(force_grams, 1);  // 1 decimal place precision
}

// =============================================================================
// CALIBRATION NOTES
// =============================================================================

/*

YOUR LOAD CELL CALIBRATION VALUE: -25.19

This value was obtained by:
  1. Placing known weight on load cell
  2. Measuring raw HX711 ADC output
  3. Dividing raw value by weight in grams
  4. Result is CALIBRATION_FACTOR

NEGATIVE SIGN MEANS:
  - Load cell output inverted (common with some installations)
  - As weight increases, ADC decreases
  - HX711 library handles this automatically

TO VERIFY CALIBRATION:
  1. Open Serial Monitor at 115200 baud
  2. Place empty: should read ~0.0g
  3. Place 100g weight: should read ~100.0g
  4. Place 500g weight: should read ~500.0g
  5. Adjust CALIBRATION_FACTOR if readings are off

ENVIRONMENTAL FACTORS:
  - Temperature: Load cells drift ~0.02%/°C
  - Drift: Over time, zero offset may change (creep)
  - Humidity: Sealed cells not affected much
  - Vibration: Can cause momentary spikes in readings

TROUBLESHOOTING:

Problem: Negative force readings
  Solution: CALIBRATION_FACTOR sign is correct (already negative)
  
Problem: Readings way off (10x too high or low)
  Solution: Check CALIBRATION_FACTOR value
  - If 10x too high: multiply CALIBRATION_FACTOR by 10
  - If 10x too low: divide CALIBRATION_FACTOR by 10
  
Problem: Drifting zero (reads increasing force with no load)
  Solution: Re-tare at startup or press physical tare button
  
Problem: Noisy readings
  Solution: Check HX711 cable shielding
  - Ensure shielded cable is used
  - Keep away from motor wires
  - Add ferrite core if needed

*/

// =============================================================================
// DEBUG MODE (OPTIONAL)
// =============================================================================

/*

To enable debug output, uncomment the lines in updateProbeStateMachine()
marked with "Optional: debug message"

This will print state transitions like:
  DEBUG: PROBE_TRIGGERED
  DEBUG: ENTERING_LOCKOUT
  DEBUG: EXITING_LOCKOUT (timeout)
  DEBUG: EXITING_LOCKOUT (force_released)

Useful for verifying probe detection is working correctly.

*/

// =============================================================================
// PERFORMANCE NOTES
// =============================================================================

/*

TIMING:
  - HX711 read: ~100-150ms (hardware timing)
  - Filter calculation: < 1ms
  - State machine update: < 1ms
  - Serial transmission: ~1ms
  - Total cycle: ~100-200ms per force value
  
  Report frequency 67Hz (15ms interval) is ASYNC to read timing,
  so we only report when new data available.

SERIAL BUFFER:
  - 115200 baud = 11520 bytes/second
  - "FORCE:XXX.X\n" = ~12 bytes
  - 67Hz * 12 bytes = ~804 bytes/second (7% of bandwidth)
  - Plenty of headroom for Motion Mega commands

MEMORY USAGE:
  - FILTER_SIZE array: 12 floats = 48 bytes
  - HX711 object: ~50 bytes
  - Total: <200 bytes (plenty on Uno/Mega)

POWER:
  - HX711: ~5mA typical
  - Arduino Uno: ~40mA typical
  - Total system: ~50mA

*/

// =============================================================================
// END OF FORCE MEGA SKETCH
// =============================================================================
