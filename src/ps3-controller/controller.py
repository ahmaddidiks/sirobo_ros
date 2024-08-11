import sys
import pygame
from pygame.locals import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        self.publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
        for joystick in self.joysticks:
            joystick.init()
            self.get_logger().info(f"Connected: {joystick.get_name()}")
        self.twist = Twist()
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        # Speed constraints
        self.max_linear_speed = 1.0
        self.min_linear_speed = 0.2
        self.max_angular_speed = 1.5
        self.min_angular_speed = 0.8

        # Speed adjustment step
        self.speed_step = 0.05

        # Track previous twist values to control sending behavior
        self.prev_twist_linear = 0.0
        self.prev_twist_angular = 0.0

    def process_gamepad_input(self):
        for event in pygame.event.get():
            if event.type == JOYBUTTONDOWN:
                if event.button == 7:  # Increase linear speed
                    self.linear_speed = min(self.linear_speed + self.speed_step, self.max_linear_speed)
                elif event.button == 6:  # Increase angular speed
                    self.angular_speed = min(self.angular_speed + self.speed_step, self.max_angular_speed)
                elif event.button == 11:  # Reset speeds
                    self.linear_speed = 0.5
                    self.angular_speed = 1.0
                self.get_logger().info(f"Button {event.button} pressed")

            if event.type == JOYAXISMOTION:
                value = event.value
                if abs(value) < 0.01:
                    value = 0.0

                if event.axis == 1:  # Axis 0 controls linear speed
                    self.twist.linear.x = -1 * value * self.linear_speed # the value from tested gamepad is reversed
                elif event.axis == 0:  # Axis 1 controls angular speed
                    self.twist.angular.z = value * self.angular_speed
                elif event.axis == 4:  # Decrease linear speed
                    self.linear_speed = max(self.linear_speed - self.speed_step, self.min_linear_speed)
                elif event.axis == 5:  # Decrease angular speed
                    self.angular_speed = max(self.angular_speed - self.speed_step, self.min_angular_speed)
                self.get_logger().info(f"Axis {event.axis} moved: value {value}")

            if event.type == JOYHATMOTION:
                if event.value == (-1, 0):  # Turn left
                    self.twist.angular.z = self.angular_speed
                    self.twist.linear.x = 0.0
                elif event.value == (1, 0):  # Turn right
                    self.twist.angular.z = -self.angular_speed
                    self.twist.linear.x = 0.0
                elif event.value == (0, 1):  # Move forward
                    self.twist.linear.x = self.linear_speed
                    self.twist.angular.z = 0.0
                elif event.value == (0, -1):  # Move backward
                    self.twist.linear.x = -self.linear_speed
                    self.twist.angular.z = 0.0
                elif event.value == (0, 0):  # Stop
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                self.get_logger().info(f"Hat motion: {event.value}")

            # Send twist command if the current values differ from the previous ones
            if (self.twist.linear.x != self.prev_twist_linear or
                self.twist.angular.z != self.prev_twist_angular):
                self.publisher_.publish(self.twist)
                self.get_logger().info(f"Published: {self.twist}")

            # Update previous twist values
            self.prev_twist_linear = self.twist.linear.x
            self.prev_twist_angular = self.twist.angular.z

def main(args=None):
    rclpy.init(args=args)
    pygame.init()

    gamepad_controller = GamepadController()

    try:
        while rclpy.ok():
            gamepad_controller.process_gamepad_input()
            pygame.time.wait(10)
    except KeyboardInterrupt:
        pass
    finally:
        gamepad_controller.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        sys.exit()

if __name__ == '__main__':
    main()