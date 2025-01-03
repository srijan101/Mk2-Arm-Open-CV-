from arm_controller import RoboticArmController
import cv2
import numpy as np
import mediapipe as mp
from tkinter import ttk
import time
from config import *

class MainController:
    def __init__(self):
        # Initialize the robotic arm controller
        self.arm = RoboticArmController()
        
        # Connect tracking callback
        self.arm.tracking_callback = self.start_tracking
        
        # Replace Face Mesh with Hand tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            max_num_hands=1  # We only need to track one hand
        )
        
        self.camera_enabled = False
        self.cap = None
        
        # Update correct travel limits
        self.max_xy_travel = 50   # maximum travel distance in mm for X and Y
        self.max_z_travel = 200   # maximum travel distance in mm for Z
        self.min_travel = 0       # minimum travel distance for all axes
        self.step_size = 5        # step size for smooth motion
        self.last_movement_time = 0
        self.movement_cooldown = 0.02
        self.home_position = {'X': 25, 'Y': 25, 'Z': 50}  # Center position (25mm for X/Y, 50mm for Z)
        self.current_position = {'X': 25, 'Y': 25, 'Z': 50}  # Start at center
        
        # Add initialization flag
        self.is_initialized = False
    
    def initialize_arm_position(self):
        """Initialize arm position by homing and moving to center"""
        try:
            print("Initializing arm position...")
            
            # Home all axes
            print("Homing all axes...")
            self.arm.home_all()
            time.sleep(2)
            
            # Move X and Y to center position simultaneously
            print("Moving X and Y to center position...")
            move_cmd = f"G1 X{self.home_position['X']} Y{self.home_position['Y']} F1000"
            self.arm.send_gcode(move_cmd)
            time.sleep(1)
            
            # Move Z axis to center position
            print("Moving Z to center position...")
            self.arm.move_axis('Z', self.home_position['Z'])
            
            # Update current positions
            self.current_position['X'] = self.home_position['X']
            self.current_position['Y'] = self.home_position['Y']
            self.current_position['Z'] = self.home_position['Z']
            
            self.is_initialized = True
            print("Arm initialization complete!")
            
        except Exception as e:
            print(f"Error during initialization: {e}")
            self.is_initialized = False
    
    def start_tracking(self):
        """Start camera and initialize arm position"""
        if not self.is_initialized:
            self.initialize_arm_position()
        self.start_camera()
    
    def start_camera(self):
        """Initialize camera with specified settings"""
        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            self.camera_enabled = True
            print("Camera initialized successfully")
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            self.camera_enabled = False
    
    def ensure_within_limits(self, axis, value):
        """Ensure the target position is within axis limits"""
        if axis in ['X', 'Y']:
            return max(self.min_travel, min(value, self.max_xy_travel))
        elif axis == 'Z':
            return max(self.min_travel, min(value, self.max_z_travel))
        return value

    def calculate_step(self, current, target, axis):
        """Calculate safe step size considering limits"""
        if abs(target - current) < self.step_size:
            return 0
        
        step = self.step_size if target > current else -self.step_size
        next_pos = current + step
        
        # Check if next position would exceed limits
        if axis in ['X', 'Y']:
            if not (0 <= next_pos <= self.max_xy_travel):
                return 0
        elif axis == 'Z':
            if not (0 <= next_pos <= self.max_z_travel):
                return 0
                
        return step

    def map_palm_to_position(self, palm_pos, axis):
        """Map palm position (0-1) to axis position considering limits"""
        if axis in ['X', 'Y']:
            # Map palm position to axis range (0-50)
            mapped_pos = palm_pos * self.max_xy_travel
            return self.ensure_within_limits(axis, mapped_pos)
        elif axis == 'Z':
            # Map palm position to Z range (0-200)
            mapped_pos = palm_pos * self.max_z_travel
            return self.ensure_within_limits(axis, mapped_pos)
        return 0

    def process_hand_movement(self):
        """Process palm position and control X, Y, Z axis movement"""
        if not self.camera_enabled or self.cap is None or not self.is_initialized:
            return
            
        try:
            ret, frame = self.cap.read()
            if not ret:
                return
                
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)
            
            h, w = frame.shape[:2]
            current_time = time.time()
            
            # Draw screen division lines first (before any hand processing)
            # Vertical center line
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            # Horizontal center line
            cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
            # Initialize default values
            target_x = self.home_position['X']  # Default to home position
            target_y = self.home_position['Y']  # Default to home position
            target_z = self.home_position['Z']  # Default to home position
            
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                
                # Get palm center coordinates (0-1 range)
                palm_x = hand_landmarks.landmark[0].x
                palm_y = hand_landmarks.landmark[0].y
                
                # Use palm_y to control both X and Y together
                xy_position = self.map_palm_to_position(palm_y, 'X')  # Use Y position for both axes
                target_x = xy_position
                target_y = xy_position  # Same position as X for synchronized movement
                target_z = self.map_palm_to_position(palm_x, 'Z')
                
                # Add deadzone in the center to prevent jitter
                deadzone = 0.1  # 10% deadzone around center
                center_xy = self.max_xy_travel / 2
                center_z = self.max_z_travel / 2
                
                # Apply deadzone to both X and Y together
                if abs(xy_position - center_xy) < (self.max_xy_travel * deadzone):
                    target_x = center_xy
                    target_y = center_xy
                if abs(target_z - center_z) < (self.max_z_travel * deadzone):
                    target_z = center_z
                
                status_text = (f"Status: XY:{xy_position:.1f} Z:{target_z:.1f}")
                status_color = (0, 255, 0)
                
                # Draw position indicators on frame
                h, w = frame.shape[:2]
                cv2.circle(frame, (int(palm_x * w), int(palm_y * h)), 10, (0, 255, 255), -1)
                
            else:
                # When no hand detected, stay at current position
                target_x = self.current_position['X']
                target_y = self.current_position['Y']
                target_z = self.current_position['Z']
                status_text = "Status: No Palm Detected - Holding Position"
                status_color = (0, 0, 255)

            # Process movement if enough time has passed
            if current_time - self.last_movement_time >= self.movement_cooldown:
                try:
                    # Calculate safe steps for all axes
                    steps = {}
                    for axis, target in [('X', target_x), ('Y', target_y), ('Z', target_z)]:
                        current = self.current_position[axis]
                        step = self.calculate_step(current, target, axis)
                        if step != 0:
                            steps[axis] = step

                    if steps:
                        print(f"Moving: {steps}")
                        self.arm.send_gcode("G91")  # Relative positioning
                        time.sleep(0.05)
                        
                        # Combine movements into a single G-code command
                        move_cmd = "G1"
                        for axis, step in steps.items():
                            move_cmd += f" {axis}{step}"
                        move_cmd += " F2000"
                        self.arm.send_gcode(move_cmd)
                        time.sleep(0.1)
                        
                        self.arm.send_gcode("G90")  # Back to absolute positioning
                        time.sleep(0.05)
                        
                        # Update current positions
                        for axis, step in steps.items():
                            self.current_position[axis] = self.ensure_within_limits(
                                axis,
                                self.current_position[axis] + step
                            )
                    
                    self.last_movement_time = current_time + 0.1
                    
                except Exception as e:
                    print(f"Error during movement: {e}")
            
            # Draw visualization
            if results.multi_hand_landmarks:
                mp_drawing = mp.solutions.drawing_utils
                mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Draw screen division lines
                h, w = frame.shape[:2]
                # Vertical center line
                cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
                # Horizontal center line
                cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
            # Draw center line (home position)
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            
            # Display information
            cv2.putText(frame, f"Target - XY:{target_x:.1f} Z:{target_z:.1f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Current - XY:{self.current_position['X']:.1f} Z:{self.current_position['Z']:.1f}", 
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, status_text, (w - 250, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Display the frame
            cv2.imshow('Palm Tracking', frame) 
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                self.stop_camera()
                
        except Exception as e:
            print(f"Error in process_hand_movement: {e}")
        
        # Add control instructions
        h, w = frame.shape[:2]
        cv2.putText(frame, "ESC: Exit", (w - 100, h - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display the frame
        cv2.imshow('Palm Tracking', frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            self.stop_camera()
    
    def run(self):
        """Main run loop"""
        try:
            tracking_button = ttk.Button(
                self.arm.root,
                text="Start Hand Tracking",  # Updated button text
                command=self.start_tracking
            )
            tracking_button.grid(row=4, column=0, padx=10, pady=5, sticky="nsew")
            
            while True:
                if self.camera_enabled:
                    self.process_hand_movement()  # Changed to hand movement processing
                self.arm.root.update()
                
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.stop_camera()
            cv2.destroyAllWindows()
            if hasattr(self.arm, 'printer'):
                self.arm.printer.close()
    
    def stop_camera(self):
        """Stop and release the camera"""
        if self.cap is not None:
            self.cap.release()
        self.camera_enabled = False
        print("Camera stopped")

def main():
    controller = MainController()
    
    # Uncomment the following line when ready to add camera support
    # controller.start_camera()
    
    controller.run()

if __name__ == "__main__":
    main()
