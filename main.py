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
        
        # Update movement parameters with correct distances
        self.max_z_travel = 100  # maximum travel distance in mm for Z
        self.max_xy_travel = 50  # maximum travel distance in mm for X and Y
        self.step_size = 5  # step size for smooth motion
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
                
                # Get palm center coordinates
                palm_x = hand_landmarks.landmark[0].x
                palm_y = hand_landmarks.landmark[0].y
                
                # Determine if palm is up or down (palm_y ranges from 0 at top to 1 at bottom)
                palm_y_threshold = 0.5  # Middle of the camera view (vertical)
                palm_x_threshold = 0.5  # Middle of the camera view (horizontal)
                
                # X and Y movement based on palm height (FLIPPED LOGIC)
                if palm_y < palm_y_threshold:  # Palm is in upper half
                    target_x = 0   # Move to minimum
                    target_y = 0   # Move to minimum
                else:  # Palm is in lower half
                    target_x = 50  # Move to maximum
                    target_y = 50  # Move to maximum
                
                # Z movement based on palm left/right position
                if palm_x < palm_x_threshold:  # Palm is in left half
                    target_z = 0   # Move Z to minimum
                else:  # Palm is in right half
                    target_z = 100 # Move Z to maximum
                
                status_text = (f"Status: Palm {'UP' if palm_y < palm_y_threshold else 'DOWN'}, "
                             f"{'RIGHT' if palm_x >= palm_x_threshold else 'LEFT'} "
                             f"X:{target_x} Y:{target_y} Z:{target_z}")
                status_color = (0, 255, 0)
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
                    # Calculate steps for all axes
                    steps = {}
                    for axis, target in zip(['X', 'Y', 'Z'], [target_x, target_y, target_z]):
                        current = self.current_position[axis]
                        if abs(target - current) >= self.step_size:
                            step = self.step_size if target > current else -self.step_size
                            steps[axis] = step

                    if steps:
                        print(f"Moving: {steps}")
                        self.arm.send_gcode("G91")  # Relative positioning
                        time.sleep(0.05)
                        
                        # Combine movements into a single G-code command
                        move_cmd = "G1"
                        for axis, step in steps.items():
                            move_cmd += f" {axis}{step}"
                        move_cmd += " F1000"  # Set feedrate to 1000
                        self.arm.send_gcode(move_cmd)
                        time.sleep(0.1)
                        
                        self.arm.send_gcode("G90")  # Back to absolute positioning
                        time.sleep(0.05)
                        
                        # Update current positions
                        for axis, step in steps.items():
                            self.current_position[axis] = max(0, min(
                                self.current_position[axis] + step,
                                self.max_xy_travel if axis in ['X', 'Y'] else self.max_z_travel
                            ))
                    
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
            cv2.putText(frame, f"X:{self.current_position['X']} Y:{self.current_position['Y']} Z:{self.current_position['Z']}", 
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
