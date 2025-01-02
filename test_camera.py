import cv2
import numpy as np

def test_camera():
    print("Initializing camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    print("Camera opened successfully!")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame")
                break
                
            # Flip the frame horizontally for a mirror effect
            frame = cv2.flip(frame, 1)
            
            # Display the frame
            cv2.imshow('Camera Test', frame)
            
            # Break the loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Test completed successfully!")
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera() 