

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nanotec_nanolib import Nanolib
from .nanolib_helper import NanolibHelper
from .motor_manager import MotorManager  



class OmniWheelNode(Node):
    def __init__(self):
        """Initializes the ROS2 Node for controlling a mecanum omnidirectional base."""
        super().__init__('omni_wheel_node')
        self.nanolib_helper = NanolibHelper()
        self.nanolib_helper.setup()


        #  Default parameters
        self.default_acceleration = 0.1  # Default acceleration in m/s²
        self.default_deceleration = 4.4  # Default deceleration in m/s²
        
        # Discover and open the bus hardware
        bus_hardware_ids = self.nanolib_helper.get_bus_hardware()
        if not bus_hardware_ids:
            raise RuntimeError('No bus hardware found.')

        peak_bus_id = next(hw for hw in bus_hardware_ids if "PEAK" in hw.getName())
        bus_hw_options = self.nanolib_helper.create_bus_hardware_options(peak_bus_id)
        self.nanolib_helper.open_bus_hardware(peak_bus_id, bus_hw_options)
        self.peak_bus_id = peak_bus_id

        # Motor setup: Ensuring correct motor order
        device_ids = [
            Nanolib.DeviceId(peak_bus_id, 1, ""),
            Nanolib.DeviceId(peak_bus_id, 2, ""),
            Nanolib.DeviceId(peak_bus_id, 3, ""),
            Nanolib.DeviceId(peak_bus_id, 4, ""),
        ]
        motor_names = ["Front left", "Front right", "Back left", "Back right"]

        self.wheel_circumference = 0.0762 * 3.14159  # Wheel circumference in meters
        self.motor_manager = MotorManager(self.nanolib_helper, device_ids, motor_names, self.wheel_circumference)

        # Initialize motors with 0 velocity and default acceleration
        self.default_acceleration = 0.4  # Default acceleration in m/s²
        self.motor_manager.setup_motors(velocity_mps=0.0, acceleration_mps2=self.default_acceleration)

        # 🔹 Publishers
        
        # Set default deceleration for all motors
        self.motor_manager.set_all_deceleration(self.default_deceleration)
        
        self.wheel_state_pub = self.create_publisher(Float32MultiArray, "/movo_base/wheel_states", 10)

        # 🔹 Subscribers
        self.create_subscription(Float32MultiArray, "/movo_base/wheel_cmds", self.cmd_vel_array_callback, 10)
        self.create_subscription(Float32MultiArray, "/movo_base/set_acceleration", self.acceleration_callback, 10)
        self.create_subscription(Float32MultiArray, "/movo_base/set_deceleration", self.deceleration_callback, 10)
        # Launch monitoring thread
        self.threads = []
        t = threading.Thread(target=self.monitor_loop, daemon=True)
        t.start()
        self.threads.append(t)

        self.get_logger().info("OmniWheelNode initialized. Monitoring and velocity control active.")

    def cmd_vel_array_callback(self, msg: Float32MultiArray):
        """
        Callback to process an array of wheel velocities (m/s).
        This ensures the correct order: [Front left, Front right, Back left, Back right].
        """
        wheel_speeds = msg.data  # Expecting an array of 4 velocities
        assert len(wheel_speeds) == 4, "Expected exactly 4 wheel speeds."

        self.motor_manager.set_all_velocities(wheel_speeds)

        for motor, speed in zip(self.motor_manager.motors, wheel_speeds):
            self.get_logger().info(f"Wheel '{motor.motor_name}' set to velocity: {speed:.2f} m/s")

    def acceleration_callback(self, msg: Float32MultiArray):
        """
        Callback to update acceleration for all motors.
        Since this is an omnidirectional base, all motors use the same acceleration.
        """
        assert len(msg.data) == 1, "Expected a single acceleration value."

        new_acceleration = msg.data[0]
        self.motor_manager.set_all_accelerations(new_acceleration)

        self.get_logger().info(f"Updated acceleration to {new_acceleration:.2f} m/s² for all motors.")
    
    def deceleration_callback(self, msg: Float32MultiArray):
        """
        Callback to update deceleration for all motors.
        Expects a single deceleration value in m/s².
        """
        assert len(msg.data) == 1, "Expected a single deceleration value."
        new_deceleration = msg.data[0]
        self.motor_manager.set_all_deceleration(new_deceleration)
        self.get_logger().info(f"Updated deceleration to {new_deceleration:.2f} m/s² for all motors.")

    
    def monitor_loop(self):
        """
        Continuously reads all motor velocities and publishes them.
        """
        while rclpy.ok():
            wheel_states = self.motor_manager.read_all_velocities()
            
            # Publish wheel states
            state_msg = Float32MultiArray(data=wheel_states)
            self.wheel_state_pub.publish(state_msg)

            # Debugging log (can be disabled for performance)
            # self.get_logger().info(f"Wheel states: {wheel_states}")
            time.sleep(0.1)

    def destroy_node(self):
        """Cleanup resources when shutting down."""
        self.get_logger().info("Stopping motors and disconnecting...")

        try:
            # Force stop motors
            self.motor_manager.set_all_velocities([0.0, 0.0, 0.0, 0.0])
            time.sleep(0.1)  # Allow command to take effect

            # Ensure motors are safely stopped
            self.motor_manager.safe_stop_all()
            self.motor_manager.disconnect_all()
            self.nanolib_helper.close_bus_hardware(self.peak_bus_id)

        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")

        self.get_logger().info("Motors safely stopped and hardware disconnected.")
        super().destroy_node()


    def __del__(self):
        """Ensure safe shutdown if the destructor is called."""
        try:
            self.destroy_node()  # Stop motors if node is deleted
        except Exception as e:
            print(f"Destructor failed: {e}")




def main(args=None):
    """Main function to initialize and run the OmniWheelNode."""
    rclpy.init(args=args)
    node = OmniWheelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Stopping...")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
