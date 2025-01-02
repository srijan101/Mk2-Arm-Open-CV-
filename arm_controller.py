import tkinter as tk
from tkinter import ttk, messagebox
import serial
import time
from threading import Lock

class RoboticArmController:
    def __init__(self):
        # Initialize serial connection with your settings
        try:
            self.printer = serial.Serial(
                port='/dev/tty.usbmodem1301',
                baudrate=250000,
                timeout=1
            )
            time.sleep(2)
            
            self.command_lock = Lock()  # Add lock for thread safety
            
            # Initialize and get firmware info
            self.send_gcode('M115')
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not connect to printer: {str(e)}")
            raise
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("MK2 Robotic Arm Controller")
        
        # Initialize position variables
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.speed = 1000
        self.is_moving = False
        
        # Create control frames
        self.create_axis_controls()
        self.create_movement_controls()
        self.create_speed_control()
        
        # Send initial setup
        self.send_startup_commands()
    
    def send_gcode(self, command):
        """Send G-code command with proper termination and wait"""
        with self.command_lock:
            self.printer.write(f"{command}\n".encode())
            self.printer.flush()
            
            # Wait for acknowledgment
            response = self.printer.readline().decode().strip()
            while response.startswith('echo:') or response.startswith('debug:'):
                response = self.printer.readline().decode().strip()
            
            return response
    
    def create_axis_controls(self):
        axis_frame = ttk.LabelFrame(self.root, text="Axis Controls")
        axis_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        # Position display
        self.pos_label = ttk.Label(axis_frame, text="Position: X:0 Y:0 Z:0")
        self.pos_label.grid(row=0, column=0, columnspan=3, pady=5)
        
        # X-axis controls
        ttk.Label(axis_frame, text="X-Axis").grid(row=1, column=0, pady=5)
        ttk.Button(axis_frame, text="+X", command=lambda: self.move_axis('X', 5)).grid(row=1, column=1)
        ttk.Button(axis_frame, text="-X", command=lambda: self.move_axis('X', -5)).grid(row=1, column=2)
        
        # Y-axis controls
        ttk.Label(axis_frame, text="Y-Axis").grid(row=2, column=0, pady=5)
        ttk.Button(axis_frame, text="+Y", command=lambda: self.move_axis('Y', 5)).grid(row=2, column=1)
        ttk.Button(axis_frame, text="-Y", command=lambda: self.move_axis('Y', -5)).grid(row=2, column=2)
        
        # Z-axis controls
        ttk.Label(axis_frame, text="Z-Axis").grid(row=3, column=0, pady=5)
        ttk.Button(axis_frame, text="+Z", command=lambda: self.move_axis('Z', 5)).grid(row=3, column=1)
        ttk.Button(axis_frame, text="-Z", command=lambda: self.move_axis('Z', -5)).grid(row=3, column=2)
    
    def move_axis(self, axis, distance):
        """Improved move_axis function with better command handling"""
        if self.is_moving:
            return
            
        # Check limits
        new_pos = getattr(self, f'current_{axis.lower()}') + distance
        if axis in ['X', 'Y'] and abs(new_pos) > 50:
            messagebox.showwarning("Limit Warning", f"{axis} axis movement exceeds 50mm limit")
            return
        elif axis == 'Z' and abs(new_pos) > 300:
            messagebox.showwarning("Limit Warning", "Z axis movement exceeds 300mm limit")
            return
        
        try:
            self.is_moving = True
            
            # Send relative positioning command
            self.send_gcode("G91")  # Set to relative positioning
            
            # Send movement command
            self.send_gcode(f"G1 {axis}{distance} F{self.speed}")
            
            # Update stored position
            setattr(self, f'current_{axis.lower()}', new_pos)
            
            # Return to absolute positioning
            self.send_gcode("G90")
            
            # Update position display
            self.pos_label.config(
                text=f"Position: X:{self.current_x:.1f} Y:{self.current_y:.1f} Z:{self.current_z:.1f}"
            )
            
        except Exception as e:
            messagebox.showerror("Movement Error", f"Error moving {axis} axis: {str(e)}")
        finally:
            self.is_moving = False
    
    def create_movement_controls(self):
        movement_frame = ttk.LabelFrame(self.root, text="Movement Controls")
        movement_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        
        ttk.Button(movement_frame, text="Home All", command=self.home_all).grid(row=0, column=0, pady=5, padx=5)
        ttk.Button(movement_frame, text="Emergency Stop", command=self.emergency_stop).grid(row=0, column=1, pady=5, padx=5)
    
    def create_speed_control(self):
        speed_frame = ttk.LabelFrame(self.root, text="Speed Control")
        speed_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")
        
        self.speed_scale = ttk.Scale(
            speed_frame,
            from_=1,
            to=50,
            orient='horizontal',
            command=self.update_speed
        )
        self.speed_scale.set(10)
        self.speed_scale.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        self.speed_label = ttk.Label(speed_frame, text="Speed: 10 mm/s")
        self.speed_label.grid(row=0, column=1, padx=5, pady=5)
    
    def update_speed(self, value):
        self.speed = float(value) * 60
        self.speed_label.config(text=f"Speed: {float(value):.1f} mm/s")
    
    def send_startup_commands(self):
        commands = [
            'G21',  # Set units to millimeters
            'G90',  # Set absolute positioning
            'M203 X50 Y50 Z50',  # Set max feedrates
            'M201 X50 Y50 Z50'   # Set max acceleration
        ]
        for cmd in commands:
            self.send_gcode(cmd)
    
    def home_all(self):
        try:
            self.send_gcode('G28')
            self.current_x = 0
            self.current_y = 0
            self.current_z = 0
            self.pos_label.config(text="Position: X:0 Y:0 Z:0")
        except Exception as e:
            messagebox.showerror("Homing Error", f"Error during homing: {str(e)}")
    
    def emergency_stop(self):
        try:
            self.send_gcode('M112')
            messagebox.showwarning("Emergency Stop", "Emergency stop activated!")
        except Exception as e:
            messagebox.showerror("Emergency Stop Error", f"Error during emergency stop: {str(e)}")
    
    def run(self):
        self.root.mainloop()
    
    def __del__(self):
        if hasattr(self, 'printer'):
            self.printer.close()

if __name__ == "__main__":
    controller = RoboticArmController()
    controller.run() 