// =============================================================================
// MOTION MEGA ARDUINO SKETCH - UPDATED WITH YOUR PARAMETERS
// =============================================================================
//
// Purpose:
//   - Control 5 stepper motors (base, shoulder, elbow, wrist_pitch, wrist_roll)
//   - Read 4 limit switches (base, shoulder, elbow, wrist_pitch)
//   - Implement homing procedure
//   - Execute motion commands (MOVE, PROBE)
//   - Monitor hardware E-Stop signal from Force Mega (pin 2)
//   - Report position and status via serial
//
// Serial Protocol:
//   Input:  "HOME\n"
//           "MOVE base_steps shoulder_steps elbow_steps pitch_steps roll_steps\n"
//           "PROBE base_steps shoulder_steps elbow_steps pitch_steps roll_steps\n"
//           "ABORT\n"
//           "CLEAR_STOP\n"
//   Output: "POS:base,shoulder,elbow,pitch,roll\n"
//           "HOME_COMPLETE\n"
//           "MOVE_COMPLETE\n"
//           "PROBE_HIT:base,shoulder,elbow,pitch,roll\n"
//           "PROBE_FAILED\n"
//           "ABORT_COMPLETE\n"
//           "STOP_TRIGGERED\n"
//           "STOP_CLEARED\n"
//
// =============================================================================

#include <AccelStepper.h>

// =============================================================================
// PIN DEFINITIONS (from your robot_control.ino)
// =============================================================================

// Limit switch pins (active HIGH)
const int PIN_LIMIT_BASE = 22;
const int PIN_LIMIT_SHOULDER = 24;
const int PIN_LIMIT_ELBOW = 26;
const int PIN_LIMIT_PITCH = 28;

// Stepper motor pins (STEP, DIR)
const int PIN_BASE_STEP = 48;
const int PIN_BASE_DIR = 50;

const int PIN_SHOULDER_STEP = 44;
const int PIN_SHOULDER_DIR = 46;

const int PIN_ELBOW_STEP = 40;
const int PIN_ELBOW_DIR = 42;

const int PIN_PITCH_STEP = 36;
const int PIN_PITCH_DIR = 38;

const int PIN_ROLL_STEP = 32;
const int PIN_ROLL_DIR = 34;

// Hardware E-Stop signal from Force Mega
const int PIN_HARDWARE_STOP = 2;

// =============================================================================
// HOME OFFSETS (from your robot_control.ino)
// =============================================================================

const long HOME_OFFSET_BASE = 15300;
const long HOME_OFFSET_SHOULDER = 1075;
const long HOME_OFFSET_ELBOW = 1203;
const long HOME_OFFSET_PITCH = 35800;
const long HOME_OFFSET_ROLL = 0;

// =============================================================================
// MOTOR SPEEDS & ACCELERATIONS (from your robot_control.ino)
// =============================================================================

const float SPEED_BASE = 5000.0;
const float ACCEL_BASE = 4000.0;

const float SPEED_SHOULDER = 4500.0;
const float ACCEL_SHOULDER = 3500.0;

const float SPEED_ELBOW = 10000.0;
const float ACCEL_ELBOW = 8000.0;

const float SPEED_PITCH = 15000.0;
const float ACCEL_PITCH = 10000.0;

const float SPEED_ROLL = 12000.0;
const float ACCEL_ROLL = 6000.0;

// Probe speed divisor (slower for safe probing)
const float PROBE_SPEED_DIVISOR = 15.0;

// =============================================================================
// SOFT LIMITS (from your robot_control.ino)
// =============================================================================

const long BASE_MIN = -100000,     BASE_MAX = 100000;
const long SHOULDER_MIN = -100000, SHOULDER_MAX = 100000;
const long ELBOW_MIN = -100000,    ELBOW_MAX = 100000;
const long PITCH_MIN = -100000,    PITCH_MAX = 100000;
const long ROLL_MIN = -100000,     ROLL_MAX = 100000;

// =============================================================================
// STEPPER OBJECTS (AccelStepper library)
// =============================================================================

// Type 1 = DRIVER mode (just step/dir pins)
AccelStepper stepper_base(1, PIN_BASE_STEP, PIN_BASE_DIR);
AccelStepper stepper_shoulder(1, PIN_SHOULDER_STEP, PIN_SHOULDER_DIR);
AccelStepper stepper_elbow(1, PIN_ELBOW_STEP, PIN_ELBOW_DIR);
AccelStepper stepper_pitch(1, PIN_PITCH_STEP, PIN_PITCH_DIR);
AccelStepper stepper_roll(1, PIN_ROLL_STEP, PIN_ROLL_DIR);

// =============================================================================
// STATE VARIABLES
// =============================================================================

enum MotionState {
  STATE_IDLE,
  STATE_HOMING,
  STATE_MOVING,
  STATE_PROBING,
  STATE_STOPPED
};

MotionState current_state = STATE_IDLE;
bool hardware_stop_triggered = false;
bool abort_requested = false;
bool waiting_for_move_complete = false;

// Probe constants
const unsigned long PROBE_TIMEOUT_MS = 25000;
const float PROBE_SPEED_FACTOR = 0.7;  // Slow approach for safety

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure limit switches as inputs (active HIGH)
  pinMode(PIN_LIMIT_BASE, INPUT);
  pinMode(PIN_LIMIT_SHOULDER, INPUT);
  pinMode(PIN_LIMIT_ELBOW, INPUT);
  pinMode(PIN_LIMIT_PITCH, INPUT);
  
  // Configure hardware stop input (from Force Mega)
  pinMode(PIN_HARDWARE_STOP, INPUT);
  
  // Configure all steppers with speed and acceleration
  stepper_base.setMaxSpeed(SPEED_BASE);
  stepper_base.setAcceleration(ACCEL_BASE);
  
  stepper_shoulder.setMaxSpeed(SPEED_SHOULDER);
  stepper_shoulder.setAcceleration(ACCEL_SHOULDER);
  
  stepper_elbow.setMaxSpeed(SPEED_ELBOW);
  stepper_elbow.setAcceleration(ACCEL_ELBOW);
  
  stepper_pitch.setMaxSpeed(SPEED_PITCH);
  stepper_pitch.setAcceleration(ACCEL_PITCH);
  
  stepper_roll.setMaxSpeed(SPEED_ROLL);
  stepper_roll.setAcceleration(ACCEL_ROLL);
  
  Serial.println("Motion Mega ready");
  delay(100);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // HIGHEST PRIORITY: Check hardware stop signal from Force Mega
  if (digitalRead(PIN_HARDWARE_STOP) == HIGH && !hardware_stop_triggered) {
    hardware_stop_triggered = true;
    emergency_stop();
    Serial.println("STOP_TRIGGERED");
    report_position();
    current_state = STATE_STOPPED;
  }
  
  // Process serial commands
  if (Serial.available() > 0) {
    process_serial_command();
  }
  
  // Execute state machine
  switch (current_state) {
    case STATE_IDLE:
      // Do nothing
      break;
      
    case STATE_HOMING:
      execute_homing();
      break;
      
    case STATE_MOVING:
      execute_motion();
      break;
      
    case STATE_PROBING:
      // Probe is handled in executeProbe() blocking call
      break;
      
    case STATE_STOPPED:
      // Motors are stopped, wait for CLEAR_STOP
      break;
  }
  
  // Always run stepper drivers (unless stopped)
  if (!hardware_stop_triggered) {
    stepper_base.run();
    stepper_shoulder.run();
    stepper_elbow.run();
    stepper_pitch.run();
    stepper_roll.run();
  }
  
  // Report position periodically (100ms)
  static unsigned long last_report = 0;
  if (millis() - last_report > 100) {
    report_position();
    last_report = millis();
  }
}

// =============================================================================
// SERIAL COMMAND PROCESSING
// =============================================================================

void process_serial_command() {
  static String input_string = "";
  
  while (Serial.available() > 0) {
    char in_char = (char)Serial.read();
    
    if (in_char == '\n') {
      input_string.trim();
      
      // Any new move command clears the stop latch
      hardware_stop_triggered = false;
      
      if (input_string == "HOME") {
        current_state = STATE_HOMING;
        abort_requested = false;
      }
      
      else if (input_string.startsWith("MOVE")) {
        long target_base, target_shoulder, target_elbow, target_pitch, target_roll;
        if (sscanf(input_string.c_str(), "MOVE %ld %ld %ld %ld %ld",
                   &target_base, &target_shoulder, &target_elbow, &target_pitch, &target_roll) == 5) {
          
          if (within_soft_limits(target_base, target_shoulder, target_elbow, target_pitch, target_roll)) {
            proportional_move(target_base, target_shoulder, target_elbow, target_pitch, target_roll);
            current_state = STATE_MOVING;
            waiting_for_move_complete = true;
            abort_requested = false;
          } else {
            Serial.println("ERROR: Target outside soft limits");
          }
        }
      }
      
      else if (input_string.startsWith("PROBE")) {
        long target_base, target_shoulder, target_elbow, target_pitch, target_roll;
        if (sscanf(input_string.c_str(), "PROBE %ld %ld %ld %ld %ld",
                   &target_base, &target_shoulder, &target_elbow, &target_pitch, &target_roll) == 5) {
          
          current_state = STATE_PROBING;
          execute_probe(target_base, target_shoulder, target_elbow, target_pitch, target_roll);
        }
      }
      
      else if (input_string == "ABORT") {
        abort_requested = true;
        emergency_stop();
        Serial.println("ABORT_COMPLETE");
        current_state = STATE_IDLE;
      }
      
      else if (input_string == "CLEAR_STOP") {
        hardware_stop_triggered = false;
        waiting_for_move_complete = false;
        Serial.println("STOP_CLEARED");
        current_state = STATE_IDLE;
      }
      
      else if (input_string == "POS?") {
        report_position();
      }
      
      input_string = "";
    } else {
      if (input_string.length() < 120) {
        input_string += in_char;
      }
    }
  }
}

// =============================================================================
// HOMING PROCEDURE
// =============================================================================

void execute_homing() {
  // Use your existing universalHome approach
  // Home in sequence: shoulder, base, elbow, pitch, roll
  
  static int home_step = 0;
  static bool homing_complete = false;
  
  if (!homing_complete) {
    switch (home_step) {
      case 0:
        universal_home(stepper_shoulder, PIN_LIMIT_SHOULDER, HOME_OFFSET_SHOULDER, SPEED_SHOULDER);
        home_step++;
        break;
      case 1:
        universal_home(stepper_base, PIN_LIMIT_BASE, HOME_OFFSET_BASE, SPEED_BASE);
        home_step++;
        break;
      case 2:
        universal_home(stepper_elbow, PIN_LIMIT_ELBOW, HOME_OFFSET_ELBOW, SPEED_ELBOW);
        home_step++;
        break;
      case 3:
        universal_home(stepper_pitch, PIN_LIMIT_PITCH, HOME_OFFSET_PITCH, SPEED_PITCH);
        home_step++;
        break;
      case 4:
        stepper_roll.setCurrentPosition(0);
        home_step++;
        break;
      case 5:
        // Homing complete
        homing_complete = true;
        Serial.println("HOME_COMPLETE");
        Serial.flush();
        current_state = STATE_IDLE;
        home_step = 0;  // Reset for next homing
        homing_complete = false;
        break;
    }
  }
}

void universal_home(AccelStepper &motor, int limit_pin, long home_offset, float motor_speed) {
  /*
   * Three-phase homing procedure (from your robot_control.ino):
   * 1. Fast approach to limit at 70% speed
   * 2. Back off 1200 steps
   * 3. Slow approach to limit at 20% speed for precision
   * 4. Move to home offset position
   */
  
  // Phase 1: Fast approach
  motor.setMaxSpeed(motor_speed * 0.7);
  motor.moveTo(-1000000L);  // Move toward limit
  
  unsigned long detection_start = 0;
  bool looking_at_signal = false;
  
  while (true) {
    motor.run();
    if (digitalRead(limit_pin) == HIGH) {
      if (!looking_at_signal) {
        detection_start = millis();
        looking_at_signal = true;
      }
      if (millis() - detection_start > 20) {  // 20ms debounce
        motor.stop();
        break;
      }
    } else {
      looking_at_signal = false;
    }
  }
  
  // Phase 2: Back off
  motor.setCurrentPosition(0);
  motor.moveTo(-1200L);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
  delay(300);
  
  // Phase 3: Slow approach
  looking_at_signal = false;
  motor.setMaxSpeed(motor_speed * 0.2);
  motor.moveTo(10000L);
  
  while (true) {
    motor.run();
    if (digitalRead(limit_pin) == HIGH) {
      if (!looking_at_signal) {
        detection_start = millis();
        looking_at_signal = true;
      }
      if (millis() - detection_start > 20) {
        motor.stop();
        break;
      }
    } else {
      looking_at_signal = false;
    }
  }
  
  // Phase 4: Move to home offset
  motor.setMaxSpeed(motor_speed);
  motor.setCurrentPosition(0);
  motor.moveTo(home_offset);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
  motor.setCurrentPosition(0);
  delay(300);
}

// =============================================================================
// PROPORTIONAL MOTION (from your robot_control.ino)
// =============================================================================

void proportional_move(long b_pos, long s_pos, long e_pos, long p_pos, long r_pos) {
  /*
   * Synchronized motion: scale all motor speeds so they arrive simultaneously
   * Uses your min 25% speed factor to keep motion smooth
   */
  
  long dist_b = abs(b_pos - stepper_base.currentPosition());
  long dist_s = abs(s_pos - stepper_shoulder.currentPosition());
  long dist_e = abs(e_pos - stepper_elbow.currentPosition());
  long dist_p = abs(p_pos - stepper_pitch.currentPosition());
  long dist_r = abs(r_pos - stepper_roll.currentPosition());
  
  long max_dist = max(max(max(dist_b, dist_s), max(dist_e, dist_p)), dist_r);
  
  if (max_dist > 0) {
    float min_factor = 0.25f;  // 25% minimum speed
    
    float factor_b = max(min_factor, (float)dist_b / max_dist);
    float factor_s = max(min_factor, (float)dist_s / max_dist);
    float factor_e = max(min_factor, (float)dist_e / max_dist);
    float factor_p = max(min_factor, (float)dist_p / max_dist);
    float factor_r = max(min_factor, (float)dist_r / max_dist);
    
    stepper_base.setMaxSpeed(SPEED_BASE * factor_b);
    stepper_base.setAcceleration(ACCEL_BASE * factor_b);
    stepper_shoulder.setMaxSpeed(SPEED_SHOULDER * factor_s);
    stepper_shoulder.setAcceleration(ACCEL_SHOULDER * factor_s);
    stepper_elbow.setMaxSpeed(SPEED_ELBOW * factor_e);
    stepper_elbow.setAcceleration(ACCEL_ELBOW * factor_e);
    stepper_pitch.setMaxSpeed(SPEED_PITCH * factor_p);
    stepper_pitch.setAcceleration(ACCEL_PITCH * factor_p);
    stepper_roll.setMaxSpeed(SPEED_ROLL * factor_r);
    stepper_roll.setAcceleration(ACCEL_ROLL * factor_r);
  }
  
  stepper_base.moveTo(b_pos);
  stepper_shoulder.moveTo(s_pos);
  stepper_elbow.moveTo(e_pos);
  stepper_pitch.moveTo(p_pos);
  stepper_roll.moveTo(r_pos);
}

// =============================================================================
// MOTION EXECUTION
// =============================================================================

void execute_motion() {
  // Check if all motors reached target
  bool all_done = stepper_base.distanceToGo() == 0 &&
                  stepper_shoulder.distanceToGo() == 0 &&
                  stepper_elbow.distanceToGo() == 0 &&
                  stepper_pitch.distanceToGo() == 0 &&
                  stepper_roll.distanceToGo() == 0;
  
  if (all_done && waiting_for_move_complete) {
    waiting_for_move_complete = false;
    Serial.println("MOVE_COMPLETE");
    Serial.flush();
    current_state = STATE_IDLE;
    restore_normal_speeds();
  }
  
  // Allow abort during motion
  if (abort_requested) {
    emergency_stop();
    current_state = STATE_IDLE;
  }
}

// =============================================================================
// PROBE EXECUTION (FORCE-CONTROLLED STOPPING)
// =============================================================================

void execute_probe(long target_b, long target_s, long target_e, long target_p, long target_r) {
  /*
   * Probe sequence:
   * 1. Move toward target at slow speed (1/15x normal)
   * 2. Monitor hardware stop signal from Force Mega
   * 3. When stop signal goes HIGH: record position and report PROBE_HIT
   * 4. If reach target without stop: report PROBE_FAILED
   */
  
  Serial.println("STATUS: Autonomous Plunge Probe Initialized...");
  
  // Set slow speeds for safe probing
  stepper_base.setMaxSpeed(SPEED_BASE / PROBE_SPEED_DIVISOR);
  stepper_base.setAcceleration(ACCEL_BASE * 3.0);
  stepper_shoulder.setMaxSpeed(SPEED_SHOULDER / PROBE_SPEED_DIVISOR);
  stepper_shoulder.setAcceleration(ACCEL_SHOULDER * 3.0);
  stepper_elbow.setMaxSpeed(SPEED_ELBOW / PROBE_SPEED_DIVISOR);
  stepper_elbow.setAcceleration(ACCEL_ELBOW * 3.0);
  stepper_pitch.setMaxSpeed(SPEED_PITCH / 8.0);
  stepper_pitch.setAcceleration(ACCEL_PITCH * 3.0);
  stepper_roll.setMaxSpeed(SPEED_ROLL / PROBE_SPEED_DIVISOR);
  stepper_roll.setAcceleration(ACCEL_ROLL * 3.0);
  
  // Send all steppers to target
  stepper_base.moveTo(target_b);
  stepper_shoulder.moveTo(target_s);
  stepper_elbow.moveTo(target_e);
  stepper_pitch.moveTo(target_p);
  stepper_roll.moveTo(target_r);
  
  unsigned long probe_start = millis();
  
  // Probe loop: run motors and monitor stop signal
  while (stepper_base.distanceToGo() != 0 ||
         stepper_shoulder.distanceToGo() != 0 ||
         stepper_elbow.distanceToGo() != 0 ||
         stepper_pitch.distanceToGo() != 0 ||
         stepper_roll.distanceToGo() != 0) {
    
    // Timeout check
    if (millis() - probe_start > PROBE_TIMEOUT_MS) {
      break;
    }
    
    // HIGHEST PRIORITY: Check hardware stop signal
    if (digitalRead(PIN_HARDWARE_STOP) == HIGH) {
      emergency_stop();
      hardware_stop_triggered = true;
      
      // Report probe hit with current position
      Serial.print("PROBE_HIT:");
      Serial.print(stepper_base.currentPosition()); Serial.print(",");
      Serial.print(stepper_shoulder.currentPosition()); Serial.print(",");
      Serial.print(stepper_elbow.currentPosition()); Serial.print(",");
      Serial.print(stepper_pitch.currentPosition()); Serial.print(",");
      Serial.println(stepper_roll.currentPosition());
      
      restore_normal_speeds();
      current_state = STATE_IDLE;
      return;
    }
    
    // Run stepper motors
    stepper_base.run();
    stepper_shoulder.run();
    stepper_elbow.run();
    stepper_pitch.run();
    stepper_roll.run();
  }
  
  // Probe completed without contact
  Serial.println("PROBE_FAILED");
  restore_normal_speeds();
  current_state = STATE_IDLE;
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void emergency_stop() {
  /*
   * Immediately stop all motors by setting target = current position
   * Do NOT change speed/accel here - let run() decelerate safely
   */
  stepper_base.moveTo(stepper_base.currentPosition());
  stepper_shoulder.moveTo(stepper_shoulder.currentPosition());
  stepper_elbow.moveTo(stepper_elbow.currentPosition());
  stepper_pitch.moveTo(stepper_pitch.currentPosition());
  stepper_roll.moveTo(stepper_roll.currentPosition());
  
  waiting_for_move_complete = false;
  abort_requested = false;
}

void restore_normal_speeds() {
  /*
   * Return all motors to normal speed/acceleration settings
   * Called after probe or other speed-modified operations
   */
  stepper_base.setMaxSpeed(SPEED_BASE);
  stepper_base.setAcceleration(ACCEL_BASE);
  stepper_shoulder.setMaxSpeed(SPEED_SHOULDER);
  stepper_shoulder.setAcceleration(ACCEL_SHOULDER);
  stepper_elbow.setMaxSpeed(SPEED_ELBOW);
  stepper_elbow.setAcceleration(ACCEL_ELBOW);
  stepper_pitch.setMaxSpeed(SPEED_PITCH);
  stepper_pitch.setAcceleration(ACCEL_PITCH);
  stepper_roll.setMaxSpeed(SPEED_ROLL);
  stepper_roll.setAcceleration(ACCEL_ROLL);
}

bool within_soft_limits(long b, long s, long e, long p, long r) {
  /*
   * Check if target position is within software limits
   * Prevents motor damage from extreme positions
   */
  return (b >= BASE_MIN && b <= BASE_MAX &&
          s >= SHOULDER_MIN && s <= SHOULDER_MAX &&
          e >= ELBOW_MIN && e <= ELBOW_MAX &&
          p >= PITCH_MIN && p <= PITCH_MAX &&
          r >= ROLL_MIN && r <= ROLL_MAX);
}

void report_position() {
  /*
   * Report current stepper positions in CSV format
   * Format: "POS:base,shoulder,elbow,pitch,roll\n"
   */
  Serial.print("POS:");
  Serial.print(stepper_base.currentPosition()); Serial.print(",");
  Serial.print(stepper_shoulder.currentPosition()); Serial.print(",");
  Serial.print(stepper_elbow.currentPosition()); Serial.print(",");
  Serial.print(stepper_pitch.currentPosition()); Serial.print(",");
  Serial.println(stepper_roll.currentPosition());
}

// =============================================================================
// END OF MOTION MEGA SKETCH
// =============================================================================

