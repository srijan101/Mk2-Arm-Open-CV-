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
        
        # Adjust movement parameters for smoother motion
        self.max_z_travel = 100  # maximum travel distance in mm for Z
        self.step_size = 5  # increase step size for smoother motion
        self.last_movement_time = 0
        self.movement_cooldown = 0.02  # reduce cooldown time
        self.home_position = {'Z': 50}  # Center position
        self.current_position = {'Z': 50}  # Start at center, not 0
        
        # Add initialization flag
        self.is_initialized = False
    
    def initialize_arm_position(self):
        """Initialize arm position by homing and moving to center"""
        try:
            print("Initializing arm position...")
            
            # Home all axes (sets to 0)
            print("Homing all axes...")
            self.arm.home_all()
            time.sleep(2)  # Wait for homing to complete
            
            # Move directly to center position (50mm for Z)
            print("Moving to center position...")
            self.arm.move_axis('Z', self.home_position['Z'])
            self.current_position['Z'] = self.home_position['Z']
            time.sleep(0.5)
            
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
        """Process palm position and control Z-axis movement"""
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
            
            # Initialize default values
            target_z = self.home_position['Z']  # Default to home position
            
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                
                # Get palm center and map it relative to screen center
                palm_x = hand_landmarks.landmark[0].x
                # Map palm position (0-1) to Z range (0-100), with center at 50
                target_z = int(palm_x * self.max_z_travel)
                
                # Ensure target is within bounds
                target_z = max(0, min(target_z, self.max_z_travel))
                
                status_text = f"Status: Palm at {target_z}mm"
                status_color = (0, 255, 0)
            else:
                # When no hand detected, stay at current position instead of moving
                target_z = self.current_position['Z']
                status_text = "Status: No Palm Detected - Holding Position"
                status_color = (0, 0, 255)

            # Process movement if enough time has passed
            if current_time - self.last_movement_time >= self.movement_cooldown:
                try:
                    # Z axis movement
                    current_z = self.current_position['Z']
                    if abs(target_z - current_z) >= self.step_size:
                        # Determine direction
                        step = self.step_size if target_z > current_z else -self.step_size
                        new_pos = current_z + step
                        new_pos = max(0, min(new_pos, self.max_z_travel))
                        
                        print(f"Moving Z by {step}mm")
                        self.arm.send_gcode("G91")  # Relative positioning
                        time.sleep(0.05)  # Reduced wait time
                        
                        # Increase feedrate for faster movement
                        self.arm.send_gcode(f"G1 Z{step} F1000")  # Increased speed
                        time.sleep(0.1)  # Reduced wait time
                        
                        self.arm.send_gcode("G90")  # Back to absolute positioning
                        time.sleep(0.05)  # Reduced wait time
                        
                        self.current_position['Z'] = new_pos
                    
                    self.last_movement_time = current_time + 0.1  # Reduced delay between movements
                    
                except Exception as e:
                    print(f"Error during movement: {e}")
            
            # Draw visualization
            if results.multi_hand_landmarks:
                mp_drawing = mp.solutions.drawing_utils
                mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Draw palm position indicator
                normalized_x = int(w * palm_x)
                cv2.circle(frame, (normalized_x, h//2), 10, (0, 255, 255), -1)
            
            # Draw center line (home position)
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            
            # Display information
            cv2.putText(frame, f"Target Z: {target_z}mm", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Current Z: {self.current_position['Z']}", 
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
