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
        
        # Initialize MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        self.camera_enabled = False
        self.cap = None
        
        # Head pose parameters
        self.prev_rotation = [0, 0, 0]  # pitch, yaw, roll
        self.movement_threshold = 15  # degrees
        self.movement_scale = 5  # mm per movement
        
        # Add initialization flag
        self.is_initialized = False
    
    def initialize_arm_position(self):
        """Initialize arm position by homing and moving to center"""
        print("Initializing arm position...")
        
        # Home all axes
        print("Homing all axes...")
        self.arm.home_all()
        time.sleep(2)  # Wait for homing to complete
        
        # Move to center position (20mm on each axis)
        print("Moving to center position...")
        self.arm.move_axis('X', 20)
        time.sleep(0.5)
        self.arm.move_axis('Y', 20)
        time.sleep(0.5)
        self.arm.move_axis('Z', 20)
        time.sleep(0.5)
        
        self.is_initialized = True
        print("Arm initialization complete!")
    
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
    
    def get_head_pose(self, face_landmarks, frame):
        """Calculate head pose from facial landmarks"""
        # Get image dimensions
        img_h, img_w = frame.shape[:2]
        
        # Convert landmarks to numpy array
        face_3d = []
        face_2d = []
        
        for idx, lm in enumerate(face_landmarks.landmark):
            if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                x, y = int(lm.x * img_w), int(lm.y * img_h)
                face_2d.append([x, y])
                face_3d.append([x, y, lm.z])
        
        face_2d = np.array(face_2d, dtype=np.float64)
        face_3d = np.array(face_3d, dtype=np.float64)
        
        # Camera matrix
        focal_length = 1 * img_w
        cam_matrix = np.array([[focal_length, 0, img_h / 2],
                             [0, focal_length, img_w / 2],
                             [0, 0, 1]])
        
        dist_matrix = np.zeros((4, 1), dtype=np.float64)
        
        # Solve PnP
        success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)
        
        if success:
            # Get rotational matrix
            rmat, jac = cv2.Rodrigues(rot_vec)
            
            # Get angles
            angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
            
            return angles
        return None
    
    def process_head_movement(self):
        """Process head movement and control arm accordingly"""
        if not self.camera_enabled or self.cap is None:
            print("Camera not enabled or capture object is None")
            return
            
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame")
            return
            
        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(frame_rgb)
        
        if results.multi_face_landmarks:
            face_landmarks = results.multi_face_landmarks[0]
            
            # Draw face mesh
            mp_drawing = mp.solutions.drawing_utils
            mp_drawing_styles = mp.solutions.drawing_styles
            mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=face_landmarks,
                connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_tesselation_style()
            )
            
            angles = self.get_head_pose(face_landmarks, frame)
            
            if angles is not None:
                pitch, yaw, roll = angles
                
                # Draw head pose angles on frame
                cv2.putText(frame, f"Pitch: {pitch:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Yaw: {yaw:.1f}", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Roll: {roll:.1f}", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Compare with previous rotation to detect movement
                if abs(yaw - self.prev_rotation[1]) > self.movement_threshold:
                    z_movement = self.movement_scale if yaw > self.prev_rotation[1] else -self.movement_scale
                    cv2.putText(frame, f"Z Movement: {z_movement}", (10, 120), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.arm.move_axis('Z', z_movement)
                
                if abs(pitch - self.prev_rotation[0]) > self.movement_threshold:
                    movement = self.movement_scale if pitch > self.prev_rotation[0] else -self.movement_scale
                    cv2.putText(frame, f"X/Y Movement: {movement}", (10, 150), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.arm.move_axis('X', movement)
                    self.arm.move_axis('Y', movement)
                
                # Draw direction arrows
                h, w = frame.shape[:2]
                center = (w // 2, h // 2)
                
                # Draw arrow for yaw (left/right)
                if abs(yaw - self.prev_rotation[1]) > self.movement_threshold:
                    arrow_length = 100
                    arrow_color = (0, 0, 255)  # Red
                    end_point = (center[0] + int(arrow_length * np.sign(yaw - self.prev_rotation[1])), center[1])
                    cv2.arrowedLine(frame, center, end_point, arrow_color, 3)
                
                # Draw arrow for pitch (up/down)
                if abs(pitch - self.prev_rotation[0]) > self.movement_threshold:
                    arrow_length = 100
                    arrow_color = (255, 0, 0)  # Blue
                    end_point = (center[0], center[1] + int(arrow_length * np.sign(pitch - self.prev_rotation[0])))
                    cv2.arrowedLine(frame, center, end_point, arrow_color, 3)
                
                self.prev_rotation = [pitch, yaw, roll]
                
                # Draw status box
                status_text = "Status: Active"
                cv2.putText(frame, status_text, (w - 200, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Draw status box when no face is detected
            h, w = frame.shape[:2]
            status_text = "Status: No Face Detected"
            cv2.putText(frame, status_text, (w - 200, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Add control instructions
        h, w = frame.shape[:2]
        cv2.putText(frame, "ESC: Exit", (w - 100, h - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Display the frame
        cv2.imshow('Head Tracking', frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            self.stop_camera()
    
    def run(self):
        """Main run loop"""
        try:
            # Create tracking control button in GUI
            tracking_button = ttk.Button(
                self.arm.root,
                text="Start Head Tracking",
                command=self.start_tracking
            )
            tracking_button.grid(row=4, column=0, padx=10, pady=5, sticky="nsew")
            
            while True:
                if self.camera_enabled:
                    self.process_head_movement()
                self.arm.root.update()  # Update GUI
                
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
