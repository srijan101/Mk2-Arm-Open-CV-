import tkinter as tk
from tkinter import ttk, messagebox
import serial
import time
from threading import Lock

class RoboticArmController:
    def __init__(self):
        # Initialize variables
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.speed = 1000  # Default speed
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Robotic Arm Control")
        
        # Create all control sections
        self.create_status_display()
        self.create_homing_control()
        self.create_manual_control()
        self.create_tracking_control()
        
        # Initialize serial connection
        self.serial = None
        self.connect_to_printer()

    def create_status_display(self):
        """Create status display frame"""
        status_frame = ttk.LabelFrame(self.root, text="Status")
        status_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        
        # Position display
        self.pos_label = ttk.Label(
            status_frame,
            text=f"Position: X:{self.current_x:.1f} Y:{self.current_y:.1f} Z:{self.current_z:.1f}"
        )
        self.pos_label.grid(row=0, column=0, padx=5, pady=5)

    def create_homing_control(self):
        """Create homing control frame"""
        homing_frame = ttk.LabelFrame(self.root, text="Homing")
        homing_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        home_all_btn = ttk.Button(
            homing_frame,
            text="Home All",
            command=self.home_all
        )
        home_all_btn.grid(row=0, column=0, padx=5, pady=5)

    def create_manual_control(self):
        """Create manual control frame"""
        manual_frame = ttk.LabelFrame(self.root, text="Manual Control")
        manual_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        
        # Axis control buttons
        axes = ['X', 'Y', 'Z']
        for i, axis in enumerate(axes):
            ttk.Label(manual_frame, text=f"{axis} Axis:").grid(row=i, column=0, padx=5, pady=5)
            
            ttk.Button(
                manual_frame,
                text="-",
                command=lambda a=axis: self.move_axis(a, -1)
            ).grid(row=i, column=1, padx=5, pady=5)
            
            ttk.Button(
                manual_frame,
                text="+",
                command=lambda a=axis: self.move_axis(a, 1)
            ).grid(row=i, column=2, padx=5, pady=5)

    def create_tracking_control(self):
        """Create tracking control frame"""
        tracking_frame = ttk.LabelFrame(self.root, text="Head Tracking")
        tracking_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")
        
        # Create button frame
        button_frame = ttk.Frame(tracking_frame)
        button_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.tracking_button = ttk.Button(
            button_frame,
            text="Start Tracking",
            command=self.start_tracking_callback
        )
        self.tracking_button.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.stop_tracking_button = ttk.Button(
            button_frame,
            text="Stop Tracking",
            command=self.stop_tracking_callback,
            state='disabled'
        )
        self.stop_tracking_button.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

    def start_tracking_callback(self):
        """Callback for tracking button"""
        if hasattr(self, 'tracking_callback'):
            print("Start tracking button pressed")
            self.tracking_button.config(state='disabled')
            self.stop_tracking_button.config(state='normal')
            self.tracking_callback()

    def stop_tracking_callback(self):
        """Callback for stop tracking button"""
        if hasattr(self, 'stop_tracking_callback'):
            print("Stop tracking button pressed")
            self.tracking_button.config(state='normal')
            self.stop_tracking_button.config(state='disabled')
            self.stop_tracking_callback()

    def home_all(self):
        """Home all axes"""
        self.send_gcode("G28")  # Home all axes
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.update_position_display()

    def move_axis(self, axis, direction):
        """Move specified axis by 1mm in given direction"""
        distance = direction * 1  # 1mm movement
        self.send_gcode(f"G91")  # Relative positioning
        self.send_gcode(f"G1 {axis}{distance} F{self.speed}")
        self.send_gcode(f"G90")  # Back to absolute positioning
        
        # Update current position
        if axis == 'X':
            self.current_x += distance
        elif axis == 'Y':
            self.current_y += distance
        elif axis == 'Z':
            self.current_z += distance
        
        self.update_position_display()

    def update_position_display(self):
        """Update position display label"""
        self.pos_label.config(
            text=f"Position: X:{self.current_x:.1f} Y:{self.current_y:.1f} Z:{self.current_z:.1f}"
        )

    def send_gcode(self, command):
        """Send G-code command to printer"""
        if self.serial:
            try:
                self.serial.write(f"{command}\n".encode())
                print(f"Sent: {command}")
            except Exception as e:
                print(f"Error sending command: {e}")

    def connect_to_printer(self):
        """Connect to the 3D printer"""
        try:
            import serial
            from config import SERIAL_PORT, BAUD_RATE
            
            print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud")
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Successfully connected to printer on {SERIAL_PORT}")
            
        except Exception as e:
            print(f"Failed to connect to printer: {e}")
            self.serial = None  # Ensure serial is None if connection fails

    def send_command(self, command):
        """Send a direct G-code command to the printer"""
        if hasattr(self, 'printer') and self.printer:
            self.printer.write(f"{command}\n".encode())
            response = self.printer.readline().decode().strip()
            return response
        return None

if __name__ == "__main__":
    controller = RoboticArmController()
    controller.root.mainloop() 