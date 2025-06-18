import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time


class EspBridgeNode(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')

        self.publisher_ = self.create_publisher(Int32, 'detected_image', 10)

        # Set up serial port
        self.serial_port = serial.Serial('/dev/ttyS0', baudrate=57600, timeout=1)

        # States
        self.state = 'INIT'
        self.loop_counter = 0

        # Start FSM
        self.timer = self.create_timer(0.2, self.fsm_loop)  # 5Hz update

    def fsm_loop(self):
        if self.state == 'INIT':
            self.init_esp1()
        elif self.state == 'RUNNING':
            self.running_loop()

    def send_and_wait(self, message, expected_response=None, delay=0.5, timeout=5.0):
        self.serial_port.write((message + '\n').encode())
        self.get_logger().info(f'Sent: {message}')
        start_time = self.get_clock().now().nanoseconds / 1e9
        response = ""
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < timeout:
            if self.serial_port.in_waiting:
                response = self.serial_port.readline().decode().strip()
                self.get_logger().info(f'Received: {response}')
                if expected_response is None or expected_response == response:
                    return response
            time.sleep(delay)
        return None

    def init_esp1(self):
        if self.send_and_wait("INIT ESP1", expected_response="ESP1 INIT"):
            if self.send_and_wait("ESP1 INIT CPY", timeout=10.0):
                final_resp = self.send_and_wait(expected_response="INIT SUCCESS", timeout=15.0)
                if final_resp == "INIT SUCCESS":
                    self.get_logger().info("Initialization successful.")
                    self.state = 'RUNNING'
                else:
                    self.get_logger().error("ESP1 did not report INIT SUCCESS.")
        else:
            self.get_logger().warn("Failed to init ESP1. Retrying...")

    def running_loop(self):
        command = "D 64" if self.loop_counter < 30 else "G 64"

        self.serial_port.write((command + '\n').encode())
        self.get_logger().info(f"Sent: {command}")
        time.sleep(0.5)
        self.serial_port.write(b"S\n")
        self.get_logger().info("Sent: S")

        start_time = self.get_clock().now().nanoseconds / 1e9
        timeout = 5.0  # seconds
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < timeout:
            if self.serial_port.in_waiting:
                response = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Received: {response}")

                if response.startswith("Image"):
                    try:
                        image_num = int(response.split()[-1])
                        self.publish_image_number(image_num)
                        self.get_logger().info(f"Published image: {image_num}. Exiting.")
                        rclpy.shutdown()
                        return
                    except ValueError:
                        self.get_logger().warn("Unrecognized image message.")
                break
        self.loop_counter += 1

    def publish_image_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()