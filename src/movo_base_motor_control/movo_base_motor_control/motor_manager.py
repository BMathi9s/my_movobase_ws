"""
Run joy package (made by ros), allows IO with Xbox Controller 

Run Joy to twist, IO controller => Twist Message X, Y and Angular Z

Run Mecanum Wheel Controller => Twist to Single Wheel Velocity

Run Omni wheel node => Sends Velocities to each wheel and reads back Velocity
"""

import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from nanotec_nanolib import Nanolib
from .nanolib_helper import NanolibHelper


class MotorController:
    def __init__(self, nanolib_helper, device_id, motor_name, wheel_circumference):
        """
        MotorController for Profile Velocity Mode (RPM-based control).

        :param nanolib_helper: NanolibHelper instance for motor communication
        :param device_id: Motor device ID
        :param motor_name: Motor name (for logging)
        :param wheel_circumference: Wheel circumference (meters)
        """
        self.nanolib_helper = nanolib_helper
        self.device = nanolib_helper.create_device(device_id)
        self.motor_name = motor_name
        self.wheel_circumference = wheel_circumference

    
    def connect(self):
        """Connects to the motor."""
        self.nanolib_helper.connect_device(self.device)

    def disconnect(self):
        """Disconnects from the motor."""
        try:
            self.nanolib_helper.disconnect_device(self.device)
        except Exception as e:
            if "already disconnected" not in str(e):
                raise e

    def stop_nanoj_program(self):
        """Stops any running NanoJ program on the motor."""
        self.nanolib_helper.write_number(self.device, 0, Nanolib.OdIndex(0x2300, 0x00), 32)

    def set_profile_velocity_mode(self):
        """Activates Profile Velocity Mode (Mode 3)."""
        self.nanolib_helper.write_number(self.device, 3, Nanolib.OdIndex(0x6060, 0x00), 8)

    def enable_operation(self):
        """Transitions motor to 'Operation Enabled' state."""
        for state in [6, 7, 0xF]:  # Enable voltage → Switch On → Enable Operation
            self.nanolib_helper.write_number(self.device, state, Nanolib.OdIndex(0x6040, 0x00), 16)

    def set_target_velocity(self, velocity_mps):
        """Sets the target velocity (m/s → RPM) and writes to `60FFh`."""
        clamped_velocity = max(min(velocity_mps, 20.0), -20.0)

        # Convert m/s to RPM
        velocity_rpm = (clamped_velocity * 60) / self.wheel_circumference

        # Write to Target Velocity register (60FFh)
        self.nanolib_helper.write_number(self.device, int(velocity_rpm), Nanolib.OdIndex(0x60FF, 0x00), 32)

        print(f"[{self.motor_name}] Set Velocity: {velocity_mps:.2f} m/s → {velocity_rpm:.2f} RPM")

    def stop(self):
        """Stops the motor by setting velocity to zero (`60FFh`)."""
        self.set_target_velocity(0.0)

    def safe_stop(self):
        """Safely stops the motor by setting velocity to 0 and transitioning to a safe state."""
        self.set_target_velocity(0.0)
        self.nanolib_helper.write_number(self.device, 0x6, Nanolib.OdIndex(0x6040, 0x00), 16)

    def read_velocity(self):
        """Reads actual motor velocity (`606Ch`) in RPM and converts it to m/s."""
        velocity_rpm = self.nanolib_helper.read_number(self.device, Nanolib.OdIndex(0x606C, 0x00))  # Read RPM

        # Convert RPM to m/s
        velocity_mps = (velocity_rpm * self.wheel_circumference) / 60
        if(velocity_mps>0):
            print(f"[{self.motor_name}] Velocity: {velocity_mps:.3f} m/s ({velocity_rpm:.2f} RPM)")

        return velocity_mps

    def set_acceleration(self, acceleration_mps2):
        """Sets motor acceleration (`6083h`, unit: RPM/s)."""
        acceleration_rpm_s = (acceleration_mps2 * 60) / self.wheel_circumference

        # Write acceleration (`6083h`)
        self.nanolib_helper.write_number(self.device, int(acceleration_rpm_s), Nanolib.OdIndex(0x6083, 0x00), 32)

        print(f"[{self.motor_name}] Set Acceleration: {acceleration_mps2:.2f} m/s² → {acceleration_rpm_s:.2f} RPM/s")

    def set_deceleration(self, deceleration_mps2):
        """Sets motor deceleration (`6084h`, unit: RPM/s)."""
        deceleration_rpm_s = (deceleration_mps2 * 60) / self.wheel_circumference

        # Write deceleration (`6084h`)
        self.nanolib_helper.write_number(self.device, int(deceleration_rpm_s), Nanolib.OdIndex(0x6084, 0x00), 32)

        print(f"[{self.motor_name}] Set Deceleration: {deceleration_mps2:.2f} m/s² → {deceleration_rpm_s:.2f} RPM/s")




class MotorManager:
    def __init__(self, nanolib_helper, device_ids, motor_names, wheel_circumference):
        """
        MotorManager handles multiple motors, providing control over speed, acceleration, and monitoring.

        :param nanolib_helper: NanolibHelper instance for motor communication
        :param device_ids: List of motor device IDs
        :param motor_names: List of motor names (for debugging/logging)
        :param wheel_circumference: Circumference of the wheel in meters
        :param steps_per_rev: Steps per motor revolution (default 2000 per documentation)
        """
        self.nanolib_helper = nanolib_helper
        self.motors = [
            MotorController(nanolib_helper, device_id, name, wheel_circumference)
            for device_id, name in zip(device_ids, motor_names)
        ]

    def setup_motors(self, velocity_mps=0.0, acceleration_mps2=1.0):
        """
        Initializes and sets up all motors with a default velocity and acceleration.

        :param velocity_mps: Initial velocity for all motors (default 0.0 m/s)
        :param acceleration_mps2: Acceleration limit for all motors (default 2.0 m/s²)
        """
        for motor in self.motors:
            motor.connect()
            motor.stop_nanoj_program()
            motor.set_profile_velocity_mode()
            # motor.set_acceleration(acceleration_mps2)  # Set acceleration limit
            motor.set_target_velocity(velocity_mps)  # Set initial velocity
            motor.enable_operation()

    def set_all_velocities(self, velocity_mps):
        """
        Sets the velocity for all motors.

        :param velocity_mps: List of target velocities in meters per second.
        """
        assert len(velocity_mps) == len(self.motors), f"Expected {len(self.motors)} velocities, but got {len(velocity_mps)}"
        
        for motor, velocity in zip(self.motors, velocity_mps):
            motor.set_target_velocity(velocity)


    def set_all_accelerations(self, acceleration_mps2):
        """
        Sets the acceleration for all motors.

        :param acceleration_mps2: Acceleration limit in meters per second².
        """
        for motor in self.motors:
            motor.set_acceleration(acceleration_mps2)
            
    def set_all_deceleration(self, deceleration_mps2):
        """
        Sets deceleration for all motors.
        :param deceleration_mps2: Deceleration limit in m/s².
        """
        for motor in self.motors:
            motor.set_deceleration(deceleration_mps2)


    def stop_motors(self):
        """Stops all motors immediately."""
        for motor in self.motors:
            motor.stop()

    def safe_stop_all(self):
        """Safely stops all motors by setting velocity to 0 before stopping."""
        for motor in self.motors:
            motor.safe_stop()

    def disconnect_all(self):
        """Disconnects all motors from the CAN bus."""
        for motor in self.motors:
            motor.disconnect()

    def read_all_velocities(self):
        """
        Reads and returns the actual velocity of all motors.

        :return: List of velocities (m/s) for all motors.
        """
        velocities = [motor.read_velocity() for motor in self.motors]
        return velocities
