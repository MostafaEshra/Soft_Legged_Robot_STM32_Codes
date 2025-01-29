import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Imu, Range

class ServoControlGUI(Node):
    def __init__(self):
        super().__init__('servo_control_gui')

        # ROS2 publishers
        self.servo_publisher = self.create_publisher(Int8MultiArray, '/servo_command', 10)
        self.reset_publisher = self.create_publisher(Int8MultiArray, '/reset_servos', 10)

        # ROS2 subscribers
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Range, '/tof/range', self.tof_callback, 10)

        # Tkinter GUI setup
        self.root = tk.Tk()
        self.root.title("SoftBOT Control")

        # IMU Data
        self.imu_frame = tk.LabelFrame(self.root, text="IMU Data", padx=10, pady=20)
        self.imu_frame.grid(row=0, column=0, padx=10, pady=10)

        self.orientation_label = tk.Label(self.imu_frame, text="Orientation (x, y, z, w):")
        self.orientation_label.grid(row=0, column=0)
        self.orientation_value = tk.Label(self.imu_frame, text="0, 0, 0, 0")
        self.orientation_value.grid(row=0, column=1)

        self.angular_velocity_label = tk.Label(self.imu_frame, text="Angular Velocity:")
        self.angular_velocity_label.grid(row=1, column=0)
        self.angular_velocity_value = tk.Label(self.imu_frame, text="0, 0, 0")
        self.angular_velocity_value.grid(row=1, column=1)

        self.linear_acceleration_label = tk.Label(self.imu_frame, text="Linear Acceleration:")
        self.linear_acceleration_label.grid(row=2, column=0)
        self.linear_acceleration_value = tk.Label(self.imu_frame, text="0, 0, 0")
        self.linear_acceleration_value.grid(row=2, column=1)

        # ToF Data
        self.tof_frame = tk.LabelFrame(self.root, text="ToF Data", padx=10, pady=10)
        self.tof_frame.grid(row=1, column=0, padx=10, pady=10)

        self.tof_label = tk.Label(self.tof_frame, text="Range:")
        self.tof_label.grid(row=0, column=0)
        self.tof_value = tk.Label(self.tof_frame, text="Waiting for data...")
        self.tof_value.grid(row=0, column=1)

        # Servo Control
        self.servo_frame = tk.LabelFrame(self.root, text="Servo Control", padx=10, pady=10)
        self.servo_frame.grid(row=2, column=0, padx=10, pady=10)

        self.servo_inputs = []
        for i in range(9):  # Assuming 6 servos
            tk.Label(self.servo_frame, text=f"Servo {i + 1}:").grid(row=i, column=0)
            entry = tk.Entry(self.servo_frame, width=5)
            entry.grid(row=i, column=1)
            self.servo_inputs.append(entry)

        self.send_button = tk.Button(self.servo_frame, text="Send", command=self.send_servo_values)
        self.send_button.grid(row=10, column=0, columnspan=2, pady=5)

        self.reset_button = tk.Button(self.root, text="Reset Servos to 0 deg", command=self.reset_servos)
        self.reset_button.grid(row=3, column=0, pady=10)

    def imu_callback(self, msg):
        # Update IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.orientation_value.config(text=f"{orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}, {orientation.w:.3f}")
        self.angular_velocity_value.config(text=f"{angular_velocity.x:.3f}, {angular_velocity.y:.3f}, {angular_velocity.z:.3f}")
        self.linear_acceleration_value.config(text=f"{linear_acceleration.x:.3f}, {linear_acceleration.y:.3f}, {linear_acceleration.z:.3f}")

    def tof_callback(self, msg):
        # Update ToF data
        self.tof_value.config(text=f"{msg.range} mm")

    def send_servo_values(self):
        try:
            # servo_values = []
            msg = Int8MultiArray()
            for i, entry in enumerate(self.servo_inputs):
                value = int(entry.get())
                if 0 <= value <= 200:
                    # servo_values.append(value)
                    msg.data[i] = value
                else:
                    raise ValueError(f"Servo {i + 1} value out of range!")

            
            
            self.servo_publisher.publish(msg)
            messagebox.showinfo("Success", "Servo values sent!")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def reset_servos(self):
        msg = Int8MultiArray()
        msg.data = [0] * 9  # Assuming 6 servos reset to 0
        self.reset_publisher.publish(msg)
        messagebox.showinfo("Success", "Servos reset to 0 degrees!")

    def run(self):
        def update_ros():
            rclpy.spin_once(self, timeout_sec=0.01)  # Process ROS2 messages
            self.root.after(10, update_ros)  # Call this function every 10ms

        update_ros()  # Start periodic ROS updates
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui_node = ServoControlGUI()
    try:
        gui_node.run()
    except KeyboardInterrupt:
        gui_node.get_logger().info("Shutting down GUI.")
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
