# =============================================================================
# LAPIDARY ROBOT CONTROL SYSTEM - COMPLETE FRESH BUILD
# =============================================================================
# 
# Purpose:
#   - Automated robotic polishing/grinding of polyhedral dice
#   - 5-axis arm with force-feedback control
#   - Real-time visualization and logging
#   - Manual dop changes between stages
#
# Architecture:
#   - Dual serial interfaces (motion control + force sensing)
#   - IK solver for wrist positioning
#   - State machine for cutting sequence
#   - GUI for operator control
#   - 3D visualization of arm and workpiece
#
# =============================================================================

import time
import math
import threading
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import ikpy.chain
import ikpy.link
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum

# =============================================================================
# SECTION 1: HARDWARE CONFIGURATION & CONSTANTS
# =============================================================================
# These values are based on physical measurements and calibration

# Stepper motor speed divisors (steps per degree)
SPD_BASE = 222.0
SPD_SHOULDER = 223.0
SPD_ELBOW = 408.0
SPD_WRIST_PITCH = 402.0
SPD_WRIST_ROLL = 40.0

# Arm segment lengths (mm) - measured with calipers
SHOULDER_HEIGHT = 300.0  # Distance from base to shoulder joint
UPPER_ARM_LENGTH = 250.0  # Shoulder to elbow
FOREARM_LENGTH = 160.0    # Elbow to wrist pivot
WRIST_LENGTH = 80.0       # Wrist structure (fixed)
DOP_LENGTH = 50.0         # Empty dop tip extension

# Total reach from wrist pivot to dop tip
TOTAL_WRIST_EXTENSION = WRIST_LENGTH + DOP_LENGTH

# Working parameters
LAP_CENTER_X = 300.0      # X position of grinding lap (mm)
LAP_CENTER_Y = 0.0        # Y position of grinding lap (mm)
LOAD_CLEARANCE_MM = 80.0  # Safe Z distance above lap during approach

# Grinding parameters
TARGET_CUTTING_FORCE_GRAMS = 150.0        # Desired pressure on stone
FORCE_TOUCH_THRESHOLD_GRAMS = 20.0        # Minimum detectable contact
FORCE_WARN_THRESHOLD_GRAMS = 250.0        # Yellow warning level
FORCE_ABORT_THRESHOLD_GRAMS = 320.0       # Red abort threshold
PROBE_CONTACT_THRESHOLD_GRAMS = 40.0      # Force threshold for probe detection

# Oscillation during grinding (micro-movements)
SWIVEL_SWEEP_MM = 25.0                    # Oscillation amplitude (mm)
SWIVEL_FREQUENCY_HZ = 0.4                 # Oscillation frequency

# Timing
SPARK_OUT_TIME_SEC = 15.0                 # Final light passes after reaching depth
PROBE_TIMEOUT_SEC = 30.0                  # Max time for probe to hit plate
MOVE_TIMEOUT_SEC = 45.0                   # Max time for large moves

# Serial communication
SERIAL_BAUDRATE = 115200
MOTION_PORT_DEFAULT = "COM4"
FORCE_PORT_DEFAULT = "COM3"

# Arduino pins and limits (from uploaded sketches)
LIMIT_SWITCH_PINS = {
    "base": 22,
    "shoulder": 24,
    "elbow": 26,
    "wrist_pitch": 28
}

HOME_STEP_OFFSETS = {
    "base": 15300,
    "shoulder": 1075,
    "elbow": 1203,
    "wrist_pitch": 35800,
    "wrist_roll": 0
}

# Motor speeds and accelerations (from Arduino config)
MOTOR_SPEEDS = {
    "base": 5000,
    "shoulder": 4500,
    "elbow": 10000,
    "wrist_pitch": 15000,
    "wrist_roll": 12000
}

MOTOR_ACCELS = {
    "base": 4000,
    "shoulder": 3500,
    "elbow": 8000,
    "wrist_pitch": 10000,
    "wrist_roll": 6000
}

# =============================================================================
# SECTION 2: DIE GEOMETRY PROFILES
# =============================================================================
# Each die shape has specific angles, rotations, and staging information
# These define the polyhedral cutting sequence

@dataclass
class CutStage:
    """Single stage in the cutting sequence for a die face."""
    name: str
    pitch_deg: Optional[float]  # Angle relative to horizontal
    roll_angles: List[float]    # Rotations around vertical axis (degrees)
    
DICE_PROFILES = {
    "D4": {
        "name": "Tetrahedron (D4)",
        "num_faces": 4,
        "hold_type": "face",
        "default_size_mm": 20.0,
        "radius_correction_factor": 3.0,
        "stages": [
            CutStage("Stage 1: Base Face", 0.0, [0.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 2: Side Faces", 70.53, [0.0, 120.0, 240.0])
        ]
    },
    "D6": {
        "name": "Cube (D6)",
        "num_faces": 6,
        "hold_type": "face",
        "default_size_mm": 20.0,
        "radius_correction_factor": 1.732,
        "stages": [
            CutStage("Stage 1: Top Face", 0.0, [0.0]),
            CutStage("Stage 2: Side Faces", 90.0, [0.0, 90.0, 180.0, 270.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 3: Bottom Face", 0.0, [0.0])
        ]
    },
    "D8": {
        "name": "Octahedron (D8)",
        "num_faces": 8,
        "hold_type": "corner",
        "default_size_mm": 20.0,
        "radius_correction_factor": 1.414,
        "stages": [
            CutStage("Stage 1: Top Pyramid", 35.26, [0.0, 90.0, 180.0, 270.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 2: Bottom Pyramid", 35.26, [45.0, 135.0, 225.0, 315.0])
        ]
    },
    "D10": {
        "name": "Pentagonal Trapezohedron (D10)",
        "num_faces": 10,
        "hold_type": "corner",
        "default_size_mm": 20.0,
        "radius_correction_factor": 1.350,
        "stages": [
            CutStage("Stage 1: Top Cap", 30.12, [0.0, 72.0, 144.0, 216.0, 288.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 2: Bottom Cap", 30.12, [36.0, 108.0, 180.0, 252.0, 324.0])
        ]
    },
    "D12": {
        "name": "Dodecahedron (D12)",
        "num_faces": 12,
        "hold_type": "face",
        "default_size_mm": 20.0,
        "radius_correction_factor": 1.258,
        "stages": [
            CutStage("Stage 1: Top Face", 0.0, [0.0]),
            CutStage("Stage 2: Upper Ring", 63.43, [0.0, 72.0, 144.0, 216.0, 288.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 3: Lower Ring", 63.43, [36.0, 108.0, 180.0, 252.0, 324.0]),
            CutStage("Stage 4: Bottom Face", 0.0, [0.0])
        ]
    },
    "D20": {
        "name": "Icosahedron (D20)",
        "num_faces": 20,
        "hold_type": "corner",
        "default_size_mm": 20.0,
        "radius_correction_factor": 1.218,
        "stages": [
            CutStage("Stage 1: Top Cap", 20.91, [0.0, 72.0, 144.0, 216.0, 288.0]),
            CutStage("Stage 2: Upper Mid Ring", 52.62, [36.0, 108.0, 180.0, 252.0, 324.0]),
            CutStage("DOP_FLIP", None, []),
            CutStage("Stage 3: Lower Mid Ring", 52.62, [0.0, 72.0, 144.0, 216.0, 288.0]),
            CutStage("Stage 4: Bottom Cap", 20.91, [36.0, 108.0, 180.0, 252.0, 324.0])
        ]
    }
}

print("✓ Lapidary Robot Control System initialized")

# =============================================================================
# SECTION 2: KINEMATIC CHAINS AND IK SOLVER
# =============================================================================
#
# The arm is modeled as a chain of rigid bodies connected by joints.
# Two chains are used:
#   1. arm: Full 6-DOF chain (base + shoulder + elbow + pitch + roll + dop tip)
#      Used for forward kinematics and final position verification
#   2. wrist_chain: 5-DOF chain (base + shoulder + elbow + wrist pivot only)
#      Used for inverse kinematics to position the wrist pivot
#
# The wrist pitch and roll are calculated separately from the shoulder/elbow
# angles to maintain the proper geometric relationships.

# Build the full arm kinematic chain
arm = ikpy.chain.Chain(
    name='lapidary_5axis_arm',
    links=[
        ikpy.link.OriginLink(),
        # Base: Fixed 300mm tall post (no rotation)
        ikpy.link.URDFLink(
            name="base_structure",
            origin_translation=[0, 0, SHOULDER_HEIGHT],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]  # Fixed vertical post
        ),
        # Shoulder: Pitches forward/back (rotates around Y axis)
        ikpy.link.URDFLink(
            name="shoulder_pitch",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]  # Pitch rotation
        ),
        # Elbow: Pitches forward/back (rotates around Y axis)
        ikpy.link.URDFLink(
            name="elbow_pitch",
            origin_translation=[0, 0, UPPER_ARM_LENGTH],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]  # Pitch rotation
        ),
        # Wrist: Pitches forward/back (rotates around Y axis)
        ikpy.link.URDFLink(
            name="wrist_pitch",
            origin_translation=[0, 0, FOREARM_LENGTH],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]  # Pitch rotation
        ),
        # Dop tip: End effector (no rotation)
        ikpy.link.URDFLink(
            name="dop_tip",
            origin_translation=[0, 0, TOTAL_WRIST_EXTENSION],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]  # Fixed end effector
        )
    ],
    active_links_mask=[False, False, True, True, True, False]
)

# Build the wrist-only chain for IK solving
# Used to find shoulder and elbow angles that position the wrist pivot
wrist_chain = ikpy.chain.Chain(
    name='wrist_positioning_chain',
    links=[
        ikpy.link.OriginLink(),
        # Base: Fixed 300mm tall post
        ikpy.link.URDFLink(
            name="base_structure",
            origin_translation=[0, 0, SHOULDER_HEIGHT],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]
        ),
        # Shoulder: Pitches forward/back
        ikpy.link.URDFLink(
            name="shoulder_pitch",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),
        # Elbow: Pitches forward/back
        ikpy.link.URDFLink(
            name="elbow_pitch",
            origin_translation=[0, 0, UPPER_ARM_LENGTH],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0]
        ),
        # Wrist pivot point (no roll yet, just the pivot location)
        ikpy.link.URDFLink(
            name="wrist_pivot",
            origin_translation=[0, 0, FOREARM_LENGTH],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]
        )
    ],
    active_links_mask=[False, False, True, True, False]
)

# =============================================================================
# SECTION 3: ANGLE AND STEP CONVERSION UTILITIES
# =============================================================================
#
# These functions convert between different representations:
#   - Motor steps (integer counts from stepper motors)
#   - Degrees (human-readable angles)
#   - Radians (mathematical representation for IK solver)

def convert_angles_to_steps(
    base_deg: float,
    shoulder_deg: float,
    elbow_deg: float,
    pitch_deg: float,
    roll_deg: float
) -> Tuple[int, int, int, int, int]:
    """
    Convert joint angles in degrees to stepper motor step counts.
    
    Args:
        base_deg: Horizontal rotation (degrees)
        shoulder_deg: Shoulder pitch (degrees)
        elbow_deg: Elbow pitch (degrees)
        pitch_deg: Wrist pitch (degrees)
        roll_deg: Wrist roll (degrees)
    
    Returns:
        Tuple of (base_steps, shoulder_steps, elbow_steps, pitch_steps, roll_steps)
    """
    s_base = int(round(base_deg * SPD_BASE))
    s_shoulder = int(round(shoulder_deg * SPD_SHOULDER))
    s_elbow = int(round(elbow_deg * SPD_ELBOW))
    s_pitch = int(round(-1 * pitch_deg * SPD_WRIST_PITCH))  # Negative for pitch direction
    s_roll = int(round(roll_deg * SPD_WRIST_ROLL))
    
    return s_base, s_shoulder, s_elbow, s_pitch, s_roll

def convert_steps_to_rads(
    s_base: int,
    s_shoulder: int,
    s_elbow: int,
    s_pitch: int,
    s_roll: int
) -> List[float]:
    """
    Convert stepper motor step counts to joint angles in radians.
    This is the inverse of convert_angles_to_steps.
    
    Args:
        s_base: Base stepper steps
        s_shoulder: Shoulder stepper steps
        s_elbow: Elbow stepper steps
        s_pitch: Pitch stepper steps
        s_roll: Roll stepper steps
    
    Returns:
        List of 6 floats: [0, base_rad, shoulder_rad, elbow_rad, pitch_rad, roll_rad]
        The first element is always 0 (OriginLink)
    """
    b_deg = s_base / SPD_BASE
    s_deg = s_shoulder / SPD_SHOULDER
    e_deg = s_elbow / SPD_ELBOW
    p_deg = -s_pitch / SPD_WRIST_PITCH
    r_deg = s_roll / SPD_WRIST_ROLL
    
    return [
        0.0,
        math.radians(b_deg),
        math.radians(s_deg),
        math.radians(e_deg),
        math.radians(p_deg),
        math.radians(r_deg)
    ]

def convert_degrees_to_rads(
    base_deg: float,
    shoulder_deg: float,
    elbow_deg: float,
    pitch_deg: float,
    roll_deg: float
) -> List[float]:
    """
    Convenience function: degrees -> radians directly.
    Used when we have angles in degrees from other calculations.
    """
    return [
        0.0,
        math.radians(base_deg),
        math.radians(shoulder_deg),
        math.radians(elbow_deg),
        math.radians(pitch_deg),
        math.radians(roll_deg)
    ]

# =============================================================================
# SECTION 4: DIE GEOMETRY CALCULATIONS
# =============================================================================
#
# These functions calculate the offset from the lap center to position
# the dop correctly for cutting each face at the right depth.

def calculate_face_inradius(die_size_mm: float, profile_key: str) -> float:
    """
    Calculate the inradius (center-to-face distance) for a given die.
    
    For "face-hold" dice: inradius = size / 2
    For "corner-hold" dice: inradius = (size / 2) / radius_correction_factor
    
    Args:
        die_size_mm: Bounding size of the die (mm)
        profile_key: Die type key (D4, D6, D8, etc.)
    
    Returns:
        Distance from die center to cutting surface (mm)
    """
    profile = DICE_PROFILES[profile_key]
    
    if profile["hold_type"] == "face":
        return die_size_mm / 2.0
    else:  # corner-hold
        return (die_size_mm / 2.0) / profile["radius_correction_factor"]

def calculate_wrist_pivot_target(
    lap_center_x: float,
    lap_center_y: float,
    lap_surface_z: float,
    desired_tip_z: float,
    pitch_deg: float,
    die_size_mm: float,
    profile_key: str,
    swivel_offset_y: float = 0.0
) -> Tuple[float, float, float]:
    """
    Calculate where the wrist pivot needs to be positioned so that
    the dop tip ends up at (lap_center_x, lap_center_y, desired_tip_z)
    when cutting at the specified pitch angle.
    
    This accounts for:
    - The dop pointing at 'pitch_deg' from horizontal
    - The dop being offset from the die center by 'inradius'
    - Wrist length and dop length
    - Micro-oscillation (swivel_offset_y)
    
    Args:
        lap_center_x: X coordinate of die center on lap
        lap_center_y: Y coordinate of die center on lap
        lap_surface_z: Z coordinate of grinding lap surface
        desired_tip_z: Target Z for dop tip
        pitch_deg: Cutting angle (degrees from horizontal)
        die_size_mm: Die bounding dimension
        profile_key: Die type (D4, D6, etc.)
        swivel_offset_y: Oscillation offset in Y (mm)
    
    Returns:
        Tuple (wrist_pivot_x, wrist_pivot_y, wrist_pivot_z)
    """
    # Get how far from die center the dop should be
    inradius = calculate_face_inradius(die_size_mm, profile_key)
    
    # Convert pitch to radians for trig
    pitch_rad = math.radians(pitch_deg)
    
    # Total offset from wrist pivot to dop tip
    # (wrist structure + dop length)
    total_extension = TOTAL_WRIST_EXTENSION
    
    # Vertical component: how much higher the wrist pivot must be
    # than the dop tip (negative pitch points downward)
    delta_z = total_extension * math.cos(pitch_rad)
    
    # Horizontal component: how far from the target XY position
    # the wrist pivot must be (accounts for pitch angle)
    delta_r = total_extension * math.sin(pitch_rad)
    
    # Calculate the horizontal offset direction
    # (pointing from lap center to wrist pivot)
    base_angle_rad = math.atan2(lap_center_y + swivel_offset_y, lap_center_x)
    
    # Position the wrist pivot
    target_wx = lap_center_x - delta_r * math.cos(base_angle_rad)
    target_wy = lap_center_y + swivel_offset_y - delta_r * math.sin(base_angle_rad)
    target_wz = desired_tip_z + delta_z
    
    return target_wx, target_wy, target_wz

# =============================================================================
# SECTION 5: FORWARD KINEMATICS VERIFICATION
# =============================================================================
#
# After solving IK or sending a move, we verify the actual position
# using forward kinematics on the measured joint angles.

def get_dop_tip_position(joint_rads: List[float]) -> Tuple[float, float, float]:
    """
    Given joint angles in radians, compute the 3D position of the dop tip
    using forward kinematics.
    
    Args:
        joint_rads: List of 6 angles [0, base, shoulder, elbow, pitch, roll]
    
    Returns:
        Tuple (x_mm, y_mm, z_mm) of dop tip position
    """
    fk_matrix = arm.forward_kinematics(joint_rads)
    x = float(fk_matrix[0, 3])
    y = float(fk_matrix[1, 3])
    z = float(fk_matrix[2, 3])
    return x, y, z

def get_dop_tip_z(joint_rads: List[float]) -> float:
    """Quick accessor for Z coordinate only."""
    _, _, z = get_dop_tip_position(joint_rads)
    return z

print("✓ Kinematic chains and conversion utilities loaded")

# =============================================================================
# SECTION 6: MOTION CONTROL AND SERIAL INTERFACE
# =============================================================================
#
# This section handles all communication with the Arduino boards:
#   - Motion Mega: Controls stepper motors, reads limit switches, probes load cells
#   - Force Mega: Reads real-time force from load cell sensor
#
# Communication is asynchronous with a background thread reading serial data
# and updating state variables that the main thread reads from.

class MotionState(Enum):
    """State machine for arm motion."""
    IDLE = "idle"
    HOMING = "homing"
    MOVING = "moving"
    PROBING = "probing"
    STOPPED = "stopped"
    ERROR = "error"

@dataclass
class ArmJointState:
    """Current state of the arm joints."""
    base_steps: int = 0
    shoulder_steps: int = 0
    elbow_steps: int = 0
    pitch_steps: int = 0
    roll_steps: int = 0
    
    @property
    def as_list(self) -> List[int]:
        return [self.base_steps, self.shoulder_steps, self.elbow_steps, 
                self.pitch_steps, self.roll_steps]
    
    @property
    def as_rads(self) -> List[float]:
        return convert_steps_to_rads(*self.as_list)

class RobotInterface:
    """
    Main interface to the dual-Arduino control system.
    
    Responsibilities:
    - Open and manage serial connections
    - Send motion commands to the Motion Mega
    - Receive force data from the Force Mega
    - Track current joint positions
    - Handle status events and errors
    """
    
    def __init__(
        self,
        motion_port: str = MOTION_PORT_DEFAULT,
        force_port: str = FORCE_PORT_DEFAULT,
        simulation_mode: bool = False,
        log_callback=None
    ):
        """
        Initialize the robot interface.
        
        Args:
            motion_port: Serial port for Motion Mega (e.g., "COM4")
            force_port: Serial port for Force Mega (e.g., "COM3")
            simulation_mode: If True, don't use actual serial ports
            log_callback: Optional function to log messages
        """
        self.motion_port = motion_port
        self.force_port = force_port
        self.simulation_mode = simulation_mode
        self.log_callback = log_callback or print
        
        # Serial port objects
        self.motion_serial = None
        self.force_serial = None
        
        # Current state
        self.joint_state = ArmJointState()
        self.motion_state = MotionState.IDLE
        self.latest_force_grams = 0.0
        self.latest_event = None
        self.latest_probe_result = None
        
        # Threading
        self.running = True
        self.read_thread = None
        
        # Simulation helpers
        self._sim_force_target = 0.0
        self._sim_force_current = 0.0
        self._sim_move_target = None
        self._sim_move_start_position = None
        self._sim_move_start_time = None
        self._sim_move_duration_ms = 2000  # Simulate 2-second moves
        
        # Only try to connect if not in simulation
        if not simulation_mode:
            self._connect_serial()
        
        # Start background read thread
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
        self.log("Robot interface initialized")
    
    def _connect_serial(self):
        """Attempt to open serial connections to both Arduinos."""
        try:
            self.motion_serial = serial.Serial(
                self.motion_port,
                SERIAL_BAUDRATE,
                timeout=0.05
            )
            self.log(f"Connected to Motion Mega on {self.motion_port}")
        except Exception as e:
            self.log(f"ERROR: Could not connect to Motion Mega: {e}")
            raise ConnectionError(f"Motion serial failed: {e}")
        
        try:
            self.force_serial = serial.Serial(
                self.force_port,
                SERIAL_BAUDRATE,
                timeout=0.05
            )
            self.log(f"Connected to Force Mega on {self.force_port}")
        except Exception as e:
            self.log(f"WARNING: Could not connect to Force Mega: {e}")
            self.force_serial = None
    
    def _read_loop(self):
        """
        Background thread: continuously reads from serial ports and updates state.
        
        This runs in a separate thread and reads:
        - From Motion Mega: Position updates (POS:), event notifications, probe results
        - From Force Mega: Real-time force readings (FORCE:)
        
        In simulation mode, this generates synthetic events and position updates.
        """
        while self.running:
            # Handle simulation mode
            if self.simulation_mode:
                # Simulate force gradually approaching target
                self._sim_force_current += (self._sim_force_target - self._sim_force_current) * 0.1
                self.latest_force_grams = self._sim_force_current
                
                # Simulate move progression
                self._simulate_move()
                
                time.sleep(0.01)
                continue
            
            # Read from Force Mega (if connected)
            if self.force_serial and self.force_serial.in_waiting > 0:
                try:
                    line = self.force_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("FORCE:"):
                        self.latest_force_grams = float(line.replace("FORCE:", "").strip())
                except Exception as e:
                    self.log(f"Force serial read error: {e}")
            
            # Read from Motion Mega (if connected)
            if self.motion_serial and self.motion_serial.in_waiting > 0:
                try:
                    line = self.motion_serial.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Parse position update
                    if line.startswith("POS:"):
                        parts = line.replace("POS:", "").strip().split(",")
                        if len(parts) >= 5:
                            self.joint_state = ArmJointState(
                                base_steps=int(parts[0]),
                                shoulder_steps=int(parts[1]),
                                elbow_steps=int(parts[2]),
                                pitch_steps=int(parts[3]),
                                roll_steps=int(parts[4])
                            )
                    
                    # Parse probe result
                    elif line.startswith("PROBE_HIT:"):
                        self.latest_probe_result = line
                        self.latest_event = "PROBE_HIT"
                    
                    elif "PROBE_FAILED" in line or "PROBE" in line and "FAILED" in line:
                        self.latest_probe_result = None
                        self.latest_event = "PROBE_FAILED"
                    
                    # Parse state-change events
                    elif line in [
                        "HOME_COMPLETE",
                        "MOVE_COMPLETE",
                        "ABORT_COMPLETE",
                        "STOP_TRIGGERED",
                        "STOP_CLEARED"
                    ]:
                        self.latest_event = line
                        
                        # Update motion state based on event
                        if line == "HOME_COMPLETE":
                            self.motion_state = MotionState.IDLE
                        elif line == "MOVE_COMPLETE":
                            self.motion_state = MotionState.IDLE
                        elif line == "ABORT_COMPLETE":
                            self.motion_state = MotionState.STOPPED
                        elif line == "STOP_TRIGGERED":
                            self.motion_state = MotionState.STOPPED
                
                except Exception as e:
                    self.log(f"Motion serial read error: {e}")
            
            time.sleep(0.008)
    
    def _simulate_move(self):
        """
        Simulate arm movement progression during moves.
        Used in simulation mode to generate MOVE_COMPLETE and position updates.
        """
        if self._sim_move_target is None:
            return
        
        # Check if move duration elapsed
        elapsed_ms = (time.time() - self._sim_move_start_time) * 1000.0
        progress = min(1.0, elapsed_ms / self._sim_move_duration_ms)
        
        if progress < 1.0:
            # Move in progress - interpolate position
            # Linear interpolation from current to target
            current_pos = self._sim_move_start_position or self.joint_state.as_list
            target_pos = self._sim_move_target
            
            new_pos = [
                int(current_pos[i] + (target_pos[i] - current_pos[i]) * progress)
                for i in range(5)
            ]
            
            self.joint_state = ArmJointState(
                base_steps=new_pos[0],
                shoulder_steps=new_pos[1],
                elbow_steps=new_pos[2],
                pitch_steps=new_pos[3],
                roll_steps=new_pos[4]
            )
        else:
            # Move complete
            self.joint_state = ArmJointState(
                base_steps=self._sim_move_target[0],
                shoulder_steps=self._sim_move_target[1],
                elbow_steps=self._sim_move_target[2],
                pitch_steps=self._sim_move_target[3],
                roll_steps=self._sim_move_target[4]
            )
            
            # Generate MOVE_COMPLETE event
            self.latest_event = "MOVE_COMPLETE"
            self._sim_move_target = None
            self._sim_move_start_position = None

    def _start_simulated_move(self, target_steps: List[int], timeout_sec: float):
        """
        Initialize a simulated move so that it completes before the caller's timeout
        and interpolates from a fixed starting pose.
        """
        self._sim_move_target = list(target_steps)
        self._sim_move_start_position = self.joint_state.as_list.copy()
        self._sim_move_start_time = time.time()
        self._sim_move_duration_ms = max(100, min(2000, int(timeout_sec * 800)))
    
    def log(self, message: str):
        """Log a message."""
        self.log_callback(message)
    
    def _send_to_motion(self, command: str):
        """Send a raw command to Motion Mega."""
        if self.simulation_mode:
            return
        
        if not self.motion_serial:
            self.log(f"ERROR: No motion serial connection. Cannot send: {command}")
            return
        
        try:
            self.motion_serial.write((command + "\n").encode('utf-8'))
        except Exception as e:
            self.log(f"ERROR sending to Motion Mega: {e}")
    
    # =========================================================================
    # MOTION COMMANDS
    # =========================================================================
    
    def home(self, timeout_sec: float = 120.0) -> bool:
        """
        Home all axes by moving to limit switches.
        
        In simulation mode:
        - Sets position to home offsets immediately
        - Generates HOME_COMPLETE event
        
        Args:
            timeout_sec: Maximum time to wait for homing to complete
        
        Returns:
            True if homing completed, False if timed out
        """
        self.log("Sending HOME command...")
        self.motion_state = MotionState.HOMING
        self.latest_event = None
        
        if self.simulation_mode:
            # Simulate home by setting positions to home offsets
            self.log("Simulating home sequence...")
            time.sleep(0.5)  # Simulate homing delay
            
            # Set position to home offsets (in simulation, this is home position)
            self.joint_state = ArmJointState(
                base_steps=int(HOME_STEP_OFFSETS["base"]),
                shoulder_steps=int(HOME_STEP_OFFSETS["shoulder"]),
                elbow_steps=int(HOME_STEP_OFFSETS["elbow"]),
                pitch_steps=int(HOME_STEP_OFFSETS["wrist_pitch"]),
                roll_steps=int(HOME_STEP_OFFSETS["wrist_roll"])
            )
            
            self.latest_event = "HOME_COMPLETE"
            self.motion_state = MotionState.IDLE
            self.log("Simulated home complete")
            return True
        
        # Real hardware: Send HOME command
        self._send_to_motion("HOME")
        
        # Wait for HOME_COMPLETE event
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_event == "HOME_COMPLETE":
                self.log("Homing complete")
                return True
            time.sleep(0.1)
        
        self.log(f"ERROR: Homing timed out after {timeout_sec}s")
        return False
    
    def move_to(
        self,
        base_steps: int,
        shoulder_steps: int,
        elbow_steps: int,
        pitch_steps: int,
        roll_steps: int,
        timeout_sec: float = MOVE_TIMEOUT_SEC
    ) -> bool:
        """
        Send a move command to a specific position (in stepper steps).
        
        In simulation mode:
        - Initiates simulated move with 2-second duration
        - Gradually updates position over time
        
        Args:
            base_steps, shoulder_steps, elbow_steps, pitch_steps, roll_steps: Target positions
            timeout_sec: Maximum time to wait for move to complete
        
        Returns:
            True if move completed, False if timed out or error
        """
        self.motion_state = MotionState.MOVING
        self.latest_event = None
        
        if self.simulation_mode:
            # Simulate move
            self._start_simulated_move(
                [base_steps, shoulder_steps, elbow_steps, pitch_steps, roll_steps],
                timeout_sec
            )
            
            # Wait for simulated move to complete
            start_time = time.time()
            while time.time() - start_time < timeout_sec:
                if self.latest_event == "MOVE_COMPLETE":
                    self.latest_event = None  # Clear for next move
                    self.motion_state = MotionState.IDLE
                    return True
                time.sleep(0.05)
            
            self.log(f"ERROR: Simulated move timed out after {timeout_sec}s")
            return False
        
        # Real hardware: Send MOVE command
        command = f"MOVE {base_steps} {shoulder_steps} {elbow_steps} {pitch_steps} {roll_steps}"
        self._send_to_motion(command)
        
        # Wait for MOVE_COMPLETE
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_event == "MOVE_COMPLETE":
                self.latest_event = None  # Clear for next move
                return True
            elif self.latest_event == "STOP_TRIGGERED":
                self.log("Move aborted: hardware stop triggered")
                return False
            time.sleep(0.05)
        
        self.log(f"ERROR: Move timed out after {timeout_sec}s")
        return False
    
    def move_to_angles(
        self,
        base_deg: float,
        shoulder_deg: float,
        elbow_deg: float,
        pitch_deg: float,
        roll_deg: float,
        timeout_sec: float = MOVE_TIMEOUT_SEC
    ) -> bool:
        """
        Convenience wrapper: move using angles in degrees instead of steps.
        """
        b, s, e, p, r = convert_angles_to_steps(
            base_deg, shoulder_deg, elbow_deg, pitch_deg, roll_deg
        )
        return self.move_to(b, s, e, p, r, timeout_sec)
    
    def probe(
        self,
        base_steps: int,
        shoulder_steps: int,
        elbow_steps: int,
        pitch_steps: int,
        roll_steps: int,
        timeout_sec: float = PROBE_TIMEOUT_SEC
    ) -> Optional[List[int]]:
        """
        Send a probe command. The arm will move downward until it detects
        a force spike from the load cell, indicating contact with the lap.
        
        In simulation mode:
        - Simulates probe motion and force detection
        - Returns probe contact position after simulated delay
        
        Args:
            base_steps, shoulder_steps, elbow_steps, pitch_steps, roll_steps: Probe direction/location
            timeout_sec: Maximum time to wait for probe contact
        
        Returns:
            List of stepper steps [base, shoulder, elbow, pitch, roll] at contact,
            or None if probe failed/timed out
        """
        self.motion_state = MotionState.PROBING
        self.latest_event = None
        self.latest_probe_result = None
        
        if self.simulation_mode:
            # Simulate probe: move toward target, then detect contact
            self.log("Simulating probe sequence...")
            
            # Move to probe position
            self._start_simulated_move(
                [base_steps, shoulder_steps, elbow_steps, pitch_steps, roll_steps],
                timeout_sec
            )
            
            # Simulate probe motion and contact
            start_time = time.time()
            while time.time() - start_time < timeout_sec:
                if self.latest_event == "MOVE_COMPLETE":
                    # Reached target - simulate force detection
                    self.log("Simulating contact detection...")
                    time.sleep(0.5)
                    
                    # Return current position as probe contact point
                    result = self.joint_state.as_list
                    self.motion_state = MotionState.IDLE
                    self.log(f"Probe contact detected at steps {result}")
                    return result
                
                time.sleep(0.05)
            
            self.log(f"ERROR: Probe timed out after {timeout_sec}s")
            return None
        
        # Real hardware: Send PROBE command
        command = f"PROBE {base_steps} {shoulder_steps} {elbow_steps} {pitch_steps} {roll_steps}"
        self._send_to_motion(command)
        
        # Wait for PROBE_HIT or timeout
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_event == "PROBE_HIT" and self.latest_probe_result:
                # Parse the result: "PROBE_HIT:b,s,e,p,r"
                try:
                    parts = self.latest_probe_result.split(":")[1].split(",")
                    result = [int(p) for p in parts[:5]]
                    self.log(f"Probe contact detected at steps {result}")
                    return result
                except Exception as e:
                    self.log(f"Error parsing probe result: {e}")
                    return None
            
            elif self.latest_event == "PROBE_FAILED":
                self.log("Probe failed: did not detect contact")
                return None
            
            time.sleep(0.05)
        
        self.log(f"ERROR: Probe timed out after {timeout_sec}s")
        return None
    
    def abort(self):
        """Abort current motion immediately."""
        self.log("Sending ABORT command")
        self.motion_state = MotionState.STOPPED
        
        if self.simulation_mode:
            self._sim_move_target = None
            self.latest_event = "ABORT_COMPLETE"
            return
        
        self._send_to_motion("ABORT")
    
    def clear_stop(self):
        """Clear any stop condition and prepare for next command."""
        if self.simulation_mode:
            self.latest_event = None
            return
        
        self._send_to_motion("CLEAR_STOP")
        self.latest_event = None
    
    # =========================================================================
    # SIMULATION HELPERS
    # =========================================================================
    
    def set_sim_force_target(self, force_grams: float):
        """Set target force for simulation (will be approached gradually)."""
        self._sim_force_target = force_grams
    
    # =========================================================================
    # STATE ACCESSORS
    # =========================================================================
    
    def get_current_position_rads(self) -> List[float]:
        """Get current arm position in radians."""
        return self.joint_state.as_rads
    
    def get_current_position_steps(self) -> List[int]:
        """Get current arm position in stepper steps."""
        return self.joint_state.as_list
    
    def get_dop_tip_position(self) -> Tuple[float, float, float]:
        """Get current 3D position of dop tip using FK."""
        return get_dop_tip_position(self.joint_state.as_rads)
    
    def get_dop_tip_z(self) -> float:
        """Get current Z position of dop tip."""
        return get_dop_tip_z(self.joint_state.as_rads)
    
    # =========================================================================
    # CLEANUP
    # =========================================================================
    
    def close(self):
        """Safely shutdown the robot interface."""
        self.log("Closing robot interface...")
        self.running = False
        
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        
        if self.motion_serial:
            try:
                self.motion_serial.close()
            except:
                pass
        
        if self.force_serial:
            try:
                self.force_serial.close()
            except:
                pass
        
        self.log("Robot interface closed")

print("✓ Motion control and serial interface loaded (simulation-enabled)")

# =============================================================================
# SECTION 7: CUTTING ALGORITHMS AND FORCE CONTROL
# =============================================================================
#
# This section contains the core logic for grinding die faces:
#   - Probe sequence to find and reference the lap surface
#   - Approach logic to move to correct angles and position
#   - Force-controlled plunge with micro-oscillations
#   - Spark-out (light passes) at target depth
#   - Retract to safe position
#
# All operations are guarded by abort flags and force thresholds.

class GrindingSequence:
    """
    Manages a single grinding operation on one die face.
    
    UPDATED: Add simulation speed multiplier for faster testing
    
    Typical sequence:
    1. Approach - move to safe distance above lap
    2. Plunge - lower onto lap under force control
    3. Spark-out - light passes to refine surface
    4. Retract - raise to safe position
    """
    
    def __init__(self, robot: RobotInterface, log_callback=None, sim_speed_multiplier=1.0):
        """
        Args:
            robot: RobotInterface instance
            log_callback: Optional callback for logging
            sim_speed_multiplier: Speed multiplier for simulation (e.g., 10.0 = 10x faster)
                                 Only affects grinding timing, not motion timing
        """
        self.robot = robot
        self.log_callback = log_callback or print
        self.abort_flag = False
        self.current_stage_name = ""
        self.sim_speed_multiplier = sim_speed_multiplier
    
    def log(self, message: str):
        """Log a message."""
        self.log_callback(message)
    
    def request_abort(self):
        """Signal abort to the grinding sequence."""
        self.abort_flag = True
    
    def _check_abort(self) -> bool:
        """
        Check if abort was requested or hardware stop triggered.
        Returns True if should abort, False if OK to continue.
        """
        if self.abort_flag:
            self.log("Grinding aborted by user")
            return True
        
        if self.robot.latest_event == "STOP_TRIGGERED":
            self.log("Hardware stop line triggered - aborting")
            return True
        
        return False
    
    def _check_force_safety(self) -> bool:
        """
        Check if force is within safe limits.
        Returns True if safe, False if should abort.
        """
        force = self.robot.latest_force_grams
        
        if force >= FORCE_ABORT_THRESHOLD_GRAMS:
            self.log(f"FORCE ABORT: {force:.1f}g exceeds limit {FORCE_ABORT_THRESHOLD_GRAMS}g")
            self.robot.abort()
            return False
        
        return True
    
    # =========================================================================
    # POSITION SOLVING (unchanged)
    # =========================================================================
    
    def solve_cut_position(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float,
        target_tip_z: float,
        swivel_y: float = 0.0
    ) -> Optional[Tuple[int, int, int, int, int]]:
        """
        Solve inverse kinematics to find stepper positions for a given cutting pose.
        
        Args:
            pitch_deg: Cutting angle (degrees from horizontal)
            roll_deg: Rotational position around vertical axis (degrees)
            die_size_mm: Die bounding dimension (mm)
            profile_key: Die type key (D4, D6, etc.)
            lap_surface_z: Height of grinding lap surface (mm)
            target_tip_z: Desired Z position of dop tip (mm)
            swivel_y: Oscillation offset in Y (mm)
        
        Returns:
            Tuple of stepper steps (b, s, e, p, r), or None if IK failed
        """
        try:
            # Calculate where the wrist pivot needs to be
            wrist_x, wrist_y, wrist_z = calculate_wrist_pivot_target(
                lap_center_x=LAP_CENTER_X,
                lap_center_y=LAP_CENTER_Y,
                lap_surface_z=lap_surface_z,
                desired_tip_z=target_tip_z,
                pitch_deg=pitch_deg,
                die_size_mm=die_size_mm,
                profile_key=profile_key,
                swivel_offset_y=swivel_y
            )
            
            # Solve IK to find shoulder and elbow angles
            # Use current position as initial guess for better convergence
            current_rads = self.robot.get_current_position_rads()
            initial_guess = current_rads[:5]  # First 5 elements only (wrist chain)
            
            wrist_rads = wrist_chain.inverse_kinematics(
                target_position=[wrist_x, wrist_y, wrist_z],
                initial_position=initial_guess
            )
            
            # Extract shoulder and elbow angles from IK result
            base_deg = math.degrees(wrist_rads[1])
            shoulder_deg = math.degrees(wrist_rads[2])
            elbow_deg = math.degrees(wrist_rads[3])
            
            # Calculate pitch angle: maintain proper arm geometry
            # The pitch keeps the dop pointing at the correct angle relative to ground
            local_pitch_deg = 180.0 - (shoulder_deg + elbow_deg)
            
            # Convert all angles to stepper steps
            b, s, e, p, r = convert_angles_to_steps(
                base_deg, shoulder_deg, elbow_deg, local_pitch_deg, roll_deg
            )
            
            return b, s, e, p, r
        
        except Exception as e:
            self.log(f"IK solver error: {e}")
            return None
    
    # =========================================================================
    # APPROACH PHASE (unchanged)
    # =========================================================================
    
    def move_to_approach(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float
    ) -> bool:
        """
        Move to safe approach position (high above lap, at correct pitch/roll).
        
        Args:
            pitch_deg: Cutting angle
            roll_deg: Rotational position
            die_size_mm: Die size
            profile_key: Die type
            lap_surface_z: Lap height reference
        
        Returns:
            True if move succeeded, False if failed or aborted
        """
        self.current_stage_name = f"Approach (P={pitch_deg:.1f}°, R={roll_deg:.1f}°)"
        self.log(self.current_stage_name)
        
        # Target is well above the lap
        target_z = lap_surface_z + LOAD_CLEARANCE_MM
        
        # Solve position
        steps = self.solve_cut_position(
            pitch_deg, roll_deg, die_size_mm, profile_key,
            lap_surface_z, target_z, swivel_y=0.0
        )
        
        if steps is None:
            return False
        
        # Send move command
        if not self.robot.move_to(*steps, timeout_sec=MOVE_TIMEOUT_SEC):
            self.log("Approach move failed or timed out")
            return False
        
        return True
    
    # =========================================================================
    # PLUNGE PHASE - FORCE CONTROLLED (FASTER IN SIM)
    # =========================================================================
    
    def plunge_to_depth(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float,
        target_depth_mm: float,
        target_force_grams: float = TARGET_CUTTING_FORCE_GRAMS
    ) -> bool:
        """
        Lower the dop onto the lap under force control.
        
        The arm gradually lowers, with micro-oscillations, until the desired
        cutting force is reached at the target depth.
        
        FASTER in simulation mode: step size increases by sim_speed_multiplier
        
        Args:
            pitch_deg: Cutting angle
            roll_deg: Rotational position
            die_size_mm: Die size
            profile_key: Die type
            lap_surface_z: Lap height reference
            target_depth_mm: How deep to cut below lap surface
            target_force_grams: Desired cutting force
        
        Returns:
            True if plunge succeeded, False if aborted or force limit exceeded
        """
        self.current_stage_name = f"Plunge (Target: {target_force_grams}g)"
        self.log(f"Plunging to {target_depth_mm}mm depth at {target_force_grams}g force...")
        
        # Start from safe distance and gradually lower
        current_clearance = LOAD_CLEARANCE_MM
        step_size_mm = 0.5 * self.sim_speed_multiplier  # FASTER in simulation
        plunge_start_time = time.time()
        
        while current_clearance > target_depth_mm:
            # Safety checks
            if self._check_abort():
                return False
            if not self._check_force_safety():
                return False
            
            # Read current force
            force = self.robot.latest_force_grams
            
            # Lower if force is below target
            if force < target_force_grams:
                current_clearance = max(target_depth_mm, current_clearance - step_size_mm)
            
            # Calculate oscillation (swivel)
            elapsed = time.time() - plunge_start_time
            swivel_y = SWIVEL_SWEEP_MM * math.sin(2.0 * math.pi * SWIVEL_FREQUENCY_HZ * elapsed)
            
            # Calculate position for this iteration
            tip_z = lap_surface_z + current_clearance
            steps = self.solve_cut_position(
                pitch_deg, roll_deg, die_size_mm, profile_key,
                lap_surface_z, tip_z, swivel_y=swivel_y
            )
            
            if steps is None:
                self.log("IK failed during plunge")
                return False
            
            # Send incremental move (faster timeout in sim)
            move_timeout = 5.0 / self.sim_speed_multiplier
            self.robot.move_to(*steps, timeout_sec=move_timeout)
            
            # Log progress (less verbose)
            if int(current_clearance * 10) % 5 == 0:  # Log every ~0.5mm
                self.log(f"  Clearance: {current_clearance:.1f}mm, Force: {force:.1f}g")
            
            # Sleep interval (MUCH SHORTER in simulation)
            time.sleep(0.02 / self.sim_speed_multiplier)
        
        self.log("Plunge completed")
        return True
    
    # =========================================================================
    # SPARK-OUT PHASE (MUCH FASTER IN SIM)
    # =========================================================================
    
    def spark_out(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float,
        target_depth_mm: float,
        duration_sec: float = SPARK_OUT_TIME_SEC
    ) -> bool:
        """
        Hold cutting position with light oscillations to refine the surface.
        
        FASTER in simulation: duration reduced by sim_speed_multiplier
        
        Args:
            pitch_deg, roll_deg, die_size_mm, profile_key, lap_surface_z, target_depth_mm: Position params
            duration_sec: How long to perform spark-out
        
        Returns:
            True if completed, False if aborted or error
        """
        # Reduce duration in simulation
        actual_duration = duration_sec / self.sim_speed_multiplier
        
        self.current_stage_name = f"Spark-out ({actual_duration:.1f}s)"
        self.log(f"Spark-out for {actual_duration:.1f} seconds...")
        
        spark_start = time.time()
        plunge_start_time = time.time()  # Track total elapsed for swivel phase
        
        while time.time() - spark_start < actual_duration:
            if self._check_abort():
                return False
            if not self._check_force_safety():
                return False
            
            # Micro-oscillation
            elapsed = time.time() - plunge_start_time
            swivel_y = SWIVEL_SWEEP_MM * math.sin(2.0 * math.pi * SWIVEL_FREQUENCY_HZ * elapsed)
            
            tip_z = lap_surface_z + target_depth_mm
            steps = self.solve_cut_position(
                pitch_deg, roll_deg, die_size_mm, profile_key,
                lap_surface_z, tip_z, swivel_y=swivel_y
            )
            
            if steps is None:
                return False
            
            move_timeout = 1.0 / self.sim_speed_multiplier
            self.robot.move_to(*steps, timeout_sec=move_timeout)
            
            time_left = actual_duration - (time.time() - spark_start)
            if int(time_left * 2) % 2 == 0:  # Log every ~0.5s
                self.log(f"  {time_left:.1f}s remaining")
            
            # MUCH SHORTER sleep in simulation
            time.sleep(0.03 / self.sim_speed_multiplier)
        
        self.log("Spark-out completed")
        return True
    
    # =========================================================================
    # RETRACT PHASE (unchanged)
    # =========================================================================
    
    def retract_to_safe(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float
    ) -> bool:
        """
        Raise the dop back to safe distance above lap.
        
        Returns:
            True if move succeeded, False if failed
        """
        self.current_stage_name = "Retract"
        self.log("Retracting to safe position...")
        
        target_z = lap_surface_z + LOAD_CLEARANCE_MM
        
        steps = self.solve_cut_position(
            pitch_deg, roll_deg, die_size_mm, profile_key,
            lap_surface_z, target_z, swivel_y=0.0
        )
        
        if steps is None:
            return False
        
        if not self.robot.move_to(*steps, timeout_sec=MOVE_TIMEOUT_SEC):
            self.log("Retract move failed or timed out")
            return False
        
        return True
    
    # =========================================================================
    # COMPLETE FACE GRIND (unchanged structure, just faster due to timeouts)
    # =========================================================================
    
    def grind_face(
        self,
        pitch_deg: float,
        roll_deg: float,
        die_size_mm: float,
        profile_key: str,
        lap_surface_z: float,
        target_depth_mm: float
    ) -> bool:
        """
        Execute complete grinding sequence for one die face:
        1. Approach
        2. Plunge to target depth
        3. Spark-out
        4. Retract
        
        Args:
            All positioning parameters
            target_depth_mm: How deep to cut below lap surface
        
        Returns:
            True if entire sequence succeeded, False if any phase failed or aborted
        """
        self.log(f"\n{'='*60}")
        self.log(f"GRINDING FACE: Pitch={pitch_deg}°, Roll={roll_deg}°")
        self.log(f"{'='*60}")
        
        # Phase 1: Approach
        if not self.move_to_approach(pitch_deg, roll_deg, die_size_mm, profile_key, lap_surface_z):
            return False
        time.sleep(0.25 / self.sim_speed_multiplier)
        
        # Phase 2: Plunge
        if not self.plunge_to_depth(pitch_deg, roll_deg, die_size_mm, profile_key,
                                    lap_surface_z, target_depth_mm):
            return False
        time.sleep(0.25 / self.sim_speed_multiplier)
        
        # Phase 3: Spark-out
        if not self.spark_out(pitch_deg, roll_deg, die_size_mm, profile_key,
                             lap_surface_z, target_depth_mm):
            return False
        time.sleep(0.25 / self.sim_speed_multiplier)
        
        # Phase 4: Retract
        if not self.retract_to_safe(pitch_deg, roll_deg, die_size_mm, profile_key, lap_surface_z):
            return False
        
        self.log(f"Face completed successfully")
        return True

print("✓ Grinding algorithms and force control loaded")

# =============================================================================
# SECTION 8: 3D VISUALIZATION AND RENDERING
# =============================================================================
#
# This section provides real-time visualization of:
#   - The arm in its current and target poses
#   - The grinding lap and die position
#   - Force feedback as a color indicator
#   - Current stage name and position info
#
# Uses matplotlib with 3D projection for interactive viewing.
class ArmVisualizer:
    """
    3D visualization of the robotic arm and workpiece.
    
    Displays:
    - Arm links in solid lines (target pose)
    - Arm links in dashed lines (actual pose)
    - Grinding lap surface
    - Die/stone position reference
    - Real-time force indicator
    - Current operation status
    """
    
    def __init__(self, figure=None):
        """
        Initialize the visualizer.
        
        Args:
            figure: Matplotlib figure to draw on (creates new one if None)
        """
        if figure is None:
            self.figure = plt.figure(figsize=(10, 8))
        else:
            self.figure = figure
        
        # Clear any existing axes
        self.figure.clear()
        
        # Create 3D axes
        self.ax = self.figure.add_subplot(111, projection='3d')
        
        # Set viewing angle (elevation, azimuth)
        self.ax.view_init(elev=25, azim=45)
        
        # Labels and limits
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_xlim([-150, 450])
        self.ax.set_ylim([-300, 300])
        self.ax.set_zlim([0, 550])
        
        # Add text annotations
        self.title_text = self.ax.set_title("Robotic Lapidary Arm", fontsize=14, fontweight='bold')
        
        # Status text box (positioned at top-left of figure)
        self.status_text = self.figure.text(
            0.02, 0.95,
            "Status: Initializing",
            fontsize=10,
            verticalalignment='top',
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        # Force indicator (positioned at top-right)
        self.force_text = self.figure.text(
            0.98, 0.95,
            "Force: 0.0 g",
            fontsize=11,
            fontweight='bold',
            horizontalalignment='right',
            verticalalignment='top',
            color='green',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.9)
        )
    
    def render(
        self,
        target_joint_rads: List[float],
        actual_joint_rads: List[float],
        lap_surface_z: float,
        current_stage: str = "",
        force_grams: float = 0.0,
        show_actual: bool = True
    ):
        """
        Render a frame showing arm pose, lap, and status.
        
        Args:
            target_joint_rads: Target joint angles in radians [0, base, shoulder, elbow, pitch, roll]
            actual_joint_rads: Actual joint angles in radians (same format)
            lap_surface_z: Z height of grinding lap surface (mm)
            current_stage: Human-readable description of current operation
            force_grams: Current force reading (grams)
            show_actual: If True, overlay actual pose with dashed lines
        """
        # Clear previous plot
        self.ax.clear()
        
        # Re-set axis properties
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_xlim([-150, 450])
        self.ax.set_ylim([-300, 300])
        self.ax.set_zlim([0, 550])
        self.ax.view_init(elev=25, azim=45)
        
        # Plot target arm pose (solid blue lines)
        try:
            arm.plot(target_joint_rads, self.ax, color='blue', linewidth=2.5)
        except Exception as e:
            print(f"Error plotting target arm: {e}")
        
        # Plot actual arm pose (dashed orange lines) if different
        if show_actual and actual_joint_rads is not None:
            try:
                arm.plot(actual_joint_rads, self.ax, color='darkorange', linewidth=2.0)
                # Make lines dashed
                for line in self.ax.lines[-5:]:
                    line.set_linestyle('--')
            except Exception as e:
                print(f"Error plotting actual arm: {e}")
        
        # Plot grinding lap surface (circle)
        theta = np.linspace(0, 2 * np.pi, 30)
        lap_radius = 75.0  # mm - visual reference only
        lap_x = LAP_CENTER_X + lap_radius * np.cos(theta)
        lap_y = LAP_CENTER_Y + lap_radius * np.sin(theta)
        lap_z = np.full_like(theta, lap_surface_z)
        self.ax.plot(lap_x, lap_y, lap_z, color='red', linewidth=2, label='Lap Surface')
        
        # Add center point marker
        self.ax.scatter([LAP_CENTER_X], [LAP_CENTER_Y], [lap_surface_z], 
                       color='red', s=100, marker='x')
        
        # Get dop tip position from FK
        try:
            tip_x, tip_y, tip_z = get_dop_tip_position(target_joint_rads)
            self.ax.scatter([tip_x], [tip_y], [tip_z], color='green', s=100, marker='o', 
                           label='Dop Tip (Target)')
        except Exception as e:
            print(f"Error computing dop tip: {e}")
            tip_x, tip_y, tip_z = 0, 0, 0
        
        # Add legend
        self.ax.legend(loc='upper left', fontsize=9)
        
        # Update title with stage info
        self.title_text.set_text(f"{current_stage}\nDop Z: {tip_z:.1f} mm | Lap Z: {lap_surface_z:.1f} mm")
        
        # Update force indicator with color coding
        if force_grams >= FORCE_ABORT_THRESHOLD_GRAMS:
            force_color = 'darkred'
            force_status = "ABORT"
        elif force_grams >= FORCE_WARN_THRESHOLD_GRAMS:
            force_color = 'orange'
            force_status = "WARN"
        elif force_grams >= FORCE_TOUCH_THRESHOLD_GRAMS:
            force_color = 'blue'
            force_status = "CUTTING"
        else:
            force_color = 'green'
            force_status = "IDLE"
        
        self.force_text.set_text(f"Force: {force_grams:.1f} g [{force_status}]")
        self.force_text.set_color(force_color)
        
        # Update status text
        status_info = (
            f"Stage: {current_stage[:30]}\n"
            f"Pos: ({tip_x:.0f}, {tip_y:.0f}, {tip_z:.0f}) mm\n"
            f"Force: {force_grams:.1f} g"
        )
        self.status_text.set_text(status_info)
        
        # CRITICAL: Draw to figure
        self.figure.canvas.draw_idle()

class SimulationRecorder:
    """
    Records frames from a grinding sequence for playback/analysis.
    
    Useful for:
    - Debugging motion sequences
    - Documenting cuts
    - Verifying kinematics
    """
    
    def __init__(self):
        """Initialize the recorder."""
        self.frames = []
    
    def record_frame(
        self,
        target_rads: List[float],
        actual_rads: List[float],
        lap_z: float,
        stage: str,
        force: float
    ):
        """
        Record a single frame.
        
        Args:
            target_rads: Target joint angles
            actual_rads: Actual joint angles
            lap_z: Lap surface height
            stage: Current operation stage
            force: Current force reading
        """
        self.frames.append({
            'target_rads': target_rads.copy() if isinstance(target_rads, list) else list(target_rads),
            'actual_rads': actual_rads.copy() if isinstance(actual_rads, list) else list(actual_rads),
            'lap_z': lap_z,
            'stage': stage,
            'force': force
        })
    
    def playback(self, visualizer: ArmVisualizer, speed: float = 1.0):
        """
        Play back recorded frames.
        
        Args:
            visualizer: ArmVisualizer instance to render to
            speed: Playback speed multiplier (1.0 = real-time)
        """
        if not self.frames:
            print("No frames recorded")
            return
        
        print(f"Playing back {len(self.frames)} frames at {speed}x speed...")
        
        frame_time = 0.033 / speed  # ~30fps at 1.0x speed
        
        for i, frame in enumerate(self.frames):
            visualizer.render(
                target_joint_rads=frame['target_rads'],
                actual_joint_rads=frame['actual_rads'],
                lap_surface_z=frame['lap_z'],
                current_stage=frame['stage'],
                force_grams=frame['force']
            )
            plt.pause(frame_time)
            
            if i % 10 == 0:
                print(f"  Frame {i}/{len(self.frames)}")
        
        print("Playback complete")
    
    def clear(self):
        """Clear all recorded frames."""
        self.frames = []
    
    def save(self, filename: str):
        """
        Save recorded frames to a file (for future analysis).
        
        Args:
            filename: Path to save to
        """
        import json
        
        # Convert numpy arrays to lists for JSON serialization
        data = []
        for frame in self.frames:
            data.append({
                'target_rads': list(frame['target_rads']),
                'actual_rads': list(frame['actual_rads']),
                'lap_z': float(frame['lap_z']),
                'stage': frame['stage'],
                'force': float(frame['force'])
            })
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Saved {len(data)} frames to {filename}")

print("✓ Visualization and recording loaded")

# =============================================================================
# SECTION 9: GRAPHICAL USER INTERFACE (GUI)
# =============================================================================
#
# Tkinter-based GUI for operator control of the lapidary robot.
#
# Layout:
#   - Left panel: Control buttons, settings, and log output
#   - Right panel: 3D visualization of arm and workpiece
#
# Workflow:
#   1. Operator selects die type and size
#   2. Operator initializes robot connection
#   3. Operator runs home sequence
#   4. Operator runs probe/calibration
#   5. Operator starts cutting cycle
#   6. System handles dop flip pause and resume
#   7. Operator reviews results

class LapidaryRobotGUI(tk.Tk):
    """
    Main GUI application for the lapidary robot system.
    
    Manages:
    - User input (die type, size, ports)
    - Robot connection and lifecycle
    - Worker threads for long-running tasks
    - Visualization updates
    - Log message display
    """
    
    def __init__(self):
        """Initialize the GUI application."""
        super().__init__()
        
        # Window setup
        self.title("Robotic Lapidary Die Grinder - Control Panel")
        self.geometry("1600x900")
        
        # Application state
        self.robot = None
        self.visualizer = None
        self.grinding_sequence = None
        self.recorder = SimulationRecorder()
        self.worker_thread = None
        self.abort_flag = False
        self.pause_flag = False
        self.resume_event = threading.Event()
        self.visual_refresh_ms = 50
        
        # Calibration state
        self.lap_surface_z = 0.0
        self.is_homed = False
        self.is_calibrated = False
        
        # Build UI
        self._build_layout()
        
        # Handle window close
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.visual_refresh_ms, self._refresh_visualization)
        
        self.log("Application ready")
    
    def _build_layout(self):
        """Construct the GUI layout."""
        
        # =====================================================================
        # LEFT PANEL: Controls
        # =====================================================================
        left_frame = ttk.Frame(self, width=400)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=10, pady=10)
        left_frame.pack_propagate(False)
        
        # --- Connection Section ---
        conn_section = ttk.LabelFrame(left_frame, text=" Connection Setup ", padding=10)
        conn_section.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_section, text="Motion Port (Arduino):").pack(anchor="w")
        self.motion_port_var = tk.StringVar(value=MOTION_PORT_DEFAULT)
        ttk.Combobox(
            conn_section,
            textvariable=self.motion_port_var,
            values=["SIM", "COM3", "COM4", "COM5", "/dev/ttyUSB0"],
            state="readonly"
        ).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(conn_section, text="Force Port (Arduino):").pack(anchor="w")
        self.force_port_var = tk.StringVar(value=FORCE_PORT_DEFAULT)
        ttk.Combobox(
            conn_section,
            textvariable=self.force_port_var,
            values=["SIM", "COM3", "COM4", "COM5", "/dev/ttyUSB1"],
            state="readonly"
        ).pack(fill=tk.X, pady=(0, 10))
        
        self.btn_connect = ttk.Button(
            conn_section,
            text="🔌 Connect to Robot",
            command=self._on_connect
        )
        self.btn_connect.pack(fill=tk.X, pady=5)
        
        # --- Die Settings Section ---
        die_section = ttk.LabelFrame(left_frame, text=" Die Configuration ", padding=10)
        die_section.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(die_section, text="Die Type:").pack(anchor="w")
        self.die_type_var = tk.StringVar(value="D20")
        die_combo = ttk.Combobox(
            die_section,
            textvariable=self.die_type_var,
            values=list(DICE_PROFILES.keys()),
            state="readonly"
        )
        die_combo.pack(fill=tk.X, pady=(0, 5))
        die_combo.bind("<<ComboboxSelected>>", self._on_die_type_changed)
        
        ttk.Label(die_section, text="Die Size (mm):").pack(anchor="w")
        self.die_size_var = tk.StringVar(value="20.0")
        ttk.Entry(die_section, textvariable=self.die_size_var).pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(die_section, text="Cut Depth (mm):").pack(anchor="w")
        self.cut_depth_var = tk.StringVar(value="0.5")
        ttk.Entry(die_section, textvariable=self.cut_depth_var).pack(fill=tk.X, pady=(0, 10))
        
        # --- Machine Control Section ---
        ctrl_section = ttk.LabelFrame(left_frame, text=" Machine Control ", padding=10)
        ctrl_section.pack(fill=tk.X, pady=(0, 10))
        
        self.btn_home = ttk.Button(
            ctrl_section,
            text="🏠 Home All Axes",
            state=tk.DISABLED,
            command=self._on_home
        )
        self.btn_home.pack(fill=tk.X, pady=2)
        
        self.btn_calibrate = ttk.Button(
            ctrl_section,
            text="📍 Probe & Calibrate Lap",
            state=tk.DISABLED,
            command=self._on_calibrate
        )
        self.btn_calibrate.pack(fill=tk.X, pady=2)
        
        self.btn_start_cut = ttk.Button(
            ctrl_section,
            text="✂️ Start Die Grinding",
            state=tk.DISABLED,
            command=self._on_start_cut
        )
        self.btn_start_cut.pack(fill=tk.X, pady=2)
        
        ttk.Separator(ctrl_section, orient='horizontal').pack(fill=tk.X, pady=5)
        
        self.btn_resume = ttk.Button(
            ctrl_section,
            text="▶ RESUME (Dop Flipped)",
            state=tk.DISABLED,
            command=self._on_resume
        )
        self.btn_resume.pack(fill=tk.X, pady=2)
        
        self.btn_abort = ttk.Button(
            ctrl_section,
            text="🛑 ABORT Immediately",
            state=tk.DISABLED,
            command=self._on_abort
        )
        self.btn_abort.pack(fill=tk.X, pady=2)
        
        # --- Status Section ---
        status_section = ttk.LabelFrame(left_frame, text=" System Status ", padding=10)
        status_section.pack(fill=tk.X, pady=(0, 10))
        
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(
            status_section,
            textvariable=self.status_var,
            font=("Arial", 10, "bold")
        ).pack(anchor="w", pady=5)
        
        # --- Log Section ---
        log_section = ttk.LabelFrame(left_frame, text=" System Log ", padding=5)
        log_section.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(
            log_section,
            height=20,
            width=45,
            state=tk.DISABLED,
            font=("Courier", 8)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # =====================================================================
        # RIGHT PANEL: Visualization
        # =====================================================================
        right_frame = ttk.Frame(self)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        viz_section = ttk.LabelFrame(right_frame, text=" 3D Arm Visualization ", padding=5)
        viz_section.pack(fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure and embed in tkinter
        self.figure = plt.figure(figsize=(8, 8), dpi=100)
        self.visualizer = ArmVisualizer(self.figure)
        
        self.canvas = FigureCanvasTkAgg(self.figure, master=viz_section)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    # =========================================================================
    # LOGGING
    # =========================================================================
    
    def log(self, message: str):
        """
        Append a message to the log display.
        
        Args:
            message: Text to log
        """
        self.log_text.configure(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)
        
        # Force GUI update for responsiveness
        self.update_idletasks()
    
    def set_status(self, status: str):
        """Update the status display."""
        self.status_var.set(status)
        self.update_idletasks()

    def _get_visualization_stage(self) -> str:
        """Return the most useful text to show in the visualization title."""
        if self.grinding_sequence and self.grinding_sequence.current_stage_name:
            return self.grinding_sequence.current_stage_name
        return self.status_var.get()

    def _refresh_visualization(self):
        """Continuously render the current robot pose from the Tk main thread."""
        if self.visualizer:
            if self.robot:
                current_rads = self.robot.get_current_position_rads()
                force_grams = self.robot.latest_force_grams
            else:
                current_rads = [0.0] * 6
                force_grams = 0.0

            self.visualizer.render(
                current_rads,
                current_rads,
                self.lap_surface_z,
                self._get_visualization_stage(),
                force_grams
            )

            if hasattr(self, "canvas"):
                self.canvas.draw_idle()

        if self.winfo_exists():
            self.after(self.visual_refresh_ms, self._refresh_visualization)
    
    # =========================================================================
    # BUTTON CALLBACKS
    # =========================================================================
    
    def _on_connect(self):
        """Handle connection button pressed."""
        motion_port = self.motion_port_var.get()
        force_port = self.force_port_var.get()
        
        # Check for simulation mode
        sim_mode = (motion_port == "SIM" or force_port == "SIM")
        
        try:
            self.robot = RobotInterface(
                motion_port=motion_port,
                force_port=force_port,
                simulation_mode=sim_mode,
                log_callback=self.log
            )
        except ConnectionError as e:
            messagebox.showerror("Connection Error", str(e))
            self.log(f"Connection failed: {e}")
            return
        
        self.grinding_sequence = GrindingSequence(
            self.robot,
            log_callback=self.log,
            sim_speed_multiplier=10.0 if sim_mode else 1.0
        )
        
        # Enable machine control buttons
        self.btn_connect.config(state=tk.DISABLED)
        self.btn_home.config(state=tk.NORMAL)
        self.btn_abort.config(state=tk.NORMAL)
        
        self.set_status("Connected - Ready to Home")
        self.log(f"Robot connected on {motion_port}/{force_port}")
    
    def _on_die_type_changed(self, event=None):
        """Handle die type selection change."""
        die_type = self.die_type_var.get()
        profile = DICE_PROFILES[die_type]
        self.die_size_var.set(profile["default_size_mm"])
        self.log(f"Selected: {profile['name']}")
    
    def _on_home(self):
        """Handle home button pressed."""
        if not self.robot:
            messagebox.showwarning("Not Connected", "Robot not connected")
            return
        
        self.set_status("Homing...")
        self.abort_flag = False
        self.worker_thread = threading.Thread(target=self._task_home, daemon=True)
        self.worker_thread.start()
    
    def _on_calibrate(self):
        """Handle calibrate button pressed."""
        if not self.robot or not self.is_homed:
            messagebox.showwarning("Not Ready", "Must home first")
            return
        
        self.set_status("Calibrating...")
        self.abort_flag = False
        self.worker_thread = threading.Thread(target=self._task_calibrate, daemon=True)
        self.worker_thread.start()
    
    def _on_start_cut(self):
        """Handle start cut button pressed."""
        if not self.robot or not self.is_calibrated:
            messagebox.showwarning("Not Ready", "Must calibrate first")
            return
        
        self.set_status("Starting grinding sequence...")
        self.abort_flag = False
        self.pause_flag = False
        self.worker_thread = threading.Thread(target=self._task_cut, daemon=True)
        self.worker_thread.start()
    
    def _on_resume(self):
        """Handle resume button pressed."""
        self.pause_flag = False
        self.resume_event.set()
        self.btn_resume.config(state=tk.DISABLED)
        self.log("Resuming from pause...")
    
    def _on_abort(self):
        """Handle abort button pressed."""
        self.abort_flag = True
        if self.robot:
            self.robot.abort()
        if self.grinding_sequence:
            self.grinding_sequence.request_abort()
        self.log("ABORT requested")
    
    # =========================================================================
    # WORKER TASKS (run in background threads)
    # =========================================================================
    
    def _task_home(self):
        """Worker task: Home all axes."""
        self.log("Homing all axes...")
        
        if self.robot.home(timeout_sec=120.0):
            self.is_homed = True
            self.btn_calibrate.config(state=tk.NORMAL)
            self.set_status("Homed - Ready to Calibrate")
            self.log("✓ Homing complete")
            
        else:
            self.set_status("Homing failed")
            self.log("✗ Homing failed or timed out")
    
    def _task_calibrate(self):
        """Worker task: Probe lap and calibrate Z reference."""
        self.log("Starting probe sequence...")
        
        # Move to approach position (high, perpendicular)
        approach_z = 100.0  # Start high above suspected lap
        
        try:
            # Solve position for perpendicular approach (pitch=0°)
            steps = self.grinding_sequence.solve_cut_position(
                pitch_deg=0.0,
                roll_deg=0.0,
                die_size_mm=float(self.die_size_var.get()),
                profile_key=self.die_type_var.get(),
                lap_surface_z=0.0,
                target_tip_z=approach_z,
                swivel_y=0.0
            )
            
            if steps is None:
                self.log("✗ IK failed during probe approach")
                return
            
            self.log("Moving to probe approach position...")
            if not self.robot.move_to(*steps, timeout_sec=45.0):
                self.log("✗ Approach move failed")
                return
            
            # Probe downward until contact
            self.log("Probing downward for lap contact...")
            probe_steps = self.robot.probe(*steps, timeout_sec=30.0)
            
            if probe_steps is None:
                self.log("✗ Probe failed - no contact detected")
                return
            
            # Convert probe steps to position
            probe_rads = convert_steps_to_rads(*probe_steps)
            _, _, probe_z = get_dop_tip_position(probe_rads)
            
            self.lap_surface_z = probe_z
            self.is_calibrated = True
            
            self.btn_start_cut.config(state=tk.NORMAL)
            self.set_status(f"Calibrated - Lap at Z={self.lap_surface_z:.1f}mm")
            self.log(f"✓ Calibration complete: Lap surface at Z = {self.lap_surface_z:.2f} mm")
            
        except Exception as e:
            self.log(f"✗ Calibration error: {e}")
            self.set_status("Calibration failed")
    
    def _task_cut(self):
        """Worker task: Execute full die grinding sequence."""
        try:
            die_type = self.die_type_var.get()
            die_size = float(self.die_size_var.get())
            cut_depth = float(self.cut_depth_var.get())
            profile = DICE_PROFILES[die_type]
            
            self.log(f"\n{'='*60}")
            self.log(f"Starting {profile['name']} Grinding Sequence")
            self.log(f"Size: {die_size}mm, Depth: {cut_depth}mm, Lap Z: {self.lap_surface_z:.2f}mm")
            self.log(f"{'='*60}\n")
            
            stage_count = 0
            
            # Iterate through all stages
            for stage in profile["stages"]:
                if self.abort_flag:
                    self.log("Grinding aborted by operator")
                    break
                
                # Handle dop flip pause
                if stage.name == "DOP_FLIP":
                    stage_count += 1
                    self.log(f"\n{'!'*60}")
                    self.log("STAGE: Dop Flip Required")
                    self.log("Manual action needed: Remove stone, flip dop, re-secure stone")
                    self.log(f"{'!'*60}\n")
                    
                    # Move to safe unload position
                    self.log("Moving to dop unload position...")
                    unload_steps = self.grinding_sequence.solve_cut_position(
                        pitch_deg=0.0,
                        roll_deg=0.0,
                        die_size_mm=die_size,
                        profile_key=die_type,
                        lap_surface_z=self.lap_surface_z,
                        target_tip_z=self.lap_surface_z + LOAD_CLEARANCE_MM,
                        swivel_y=0.0
                    )
                    
                    if unload_steps:
                        self.robot.move_to(*unload_steps, timeout_sec=45.0)
                    
                    # Wait for operator to resume
                    self.pause_flag = True
                    self.after(0, lambda: self.btn_resume.config(state=tk.NORMAL))
                    self.set_status("PAUSED: Waiting for dop flip completion")
                    self.resume_event.clear()
                    self.resume_event.wait()
                    
                    if self.abort_flag:
                        self.log("Cut aborted during pause")
                        break
                    
                    self.robot.clear_stop()
                    self.log("Resuming from dop flip pause...")
                    continue
                
                # Regular cutting stage
                stage_count += 1
                self.log(f"\n--- Stage {stage_count}: {stage.name} ---")
                
                pitch = stage.pitch_deg
                
                # Grind each face in this stage
                for roll in stage.roll_angles:
                    if self.abort_flag:
                        self.log("Grinding aborted")
                        break
                    
                    self.log(f"\nGrinding face at P={pitch:.1f}°, R={roll:.1f}°")
                    
                    # Grind this face
                    success = self.grinding_sequence.grind_face(
                        pitch_deg=pitch,
                        roll_deg=roll,
                        die_size_mm=die_size,
                        profile_key=die_type,
                        lap_surface_z=self.lap_surface_z,
                        target_depth_mm=cut_depth
                    )
                    
                    if not success:
                        self.log("Face grinding failed")
                        break
                    
            # Return to home position
            if not self.abort_flag:
                self.log("\nGrinding complete! Returning home...")
                self.robot.home(timeout_sec=120.0)
            
            self.set_status("Ready")
            self.log("✓ Die grinding sequence finished")
        
        except Exception as e:
            self.log(f"✗ Grinding error: {e}")
            self.set_status(f"Error: {e}")
    
    # =========================================================================
    # WINDOW MANAGEMENT
    # =========================================================================
    
    def _on_close(self):
        """Handle window close event."""
        self.log("Closing application...")
        
        if self.robot:
            self.robot.close()
        
        self.destroy()

# =============================================================================
# MAIN APPLICATION ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    app = LapidaryRobotGUI()
    app.mainloop()