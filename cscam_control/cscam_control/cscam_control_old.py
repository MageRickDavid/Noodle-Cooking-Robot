import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gripper_interfaces.srv import MoveCscam  # Replace with your actual service definition
from gripper_interfaces.srv import ResetCscam
from gripper_interfaces.msg import CscamMotors
import socket


class BASKET:
    def __init__(self, idBasket):
        self.cooking
        self.up
        self.id = idBasket



class TCPClientNode(Node):
    def __init__(self):
        super().__init__('tcp_client_node')
        self.server_ip = "192.168.150.15"  # Replace with your server's IP
        self.server_port = 8080
        self.read_position_flag = True
       
        self.current1 = None
        self.current2 = None
        self.current3 = None
        self.current4 = None





        
        # Reduce socket timeout for faster error detection
        self.socket_timeout = 10.0  # Reduced from 10 seconds to 2 seconds
        
        self.server_setup()
        self.call_back_groups_setup()
        self.service_setup()
        self.publisher_setup()
        #self.timer_setup()
        self.get_logger().info("Cscam controller fully setup!!!")

    def call_back_groups_setup(self):
        self.move_group = ReentrantCallbackGroup()
        self.reading_servo_group = MutuallyExclusiveCallbackGroup()
        self.servoPositionPublisher_group = MutuallyExclusiveCallbackGroup()
        self.resetCscam_group = MutuallyExclusiveCallbackGroup()
        self.sinusoidal_group = ReentrantCallbackGroup()

    def publisher_setup(self):
        try:
            self.cscamPublisher = self.create_publisher(CscamMotors, f"/cscam/position",10, callback_group=self.servoPositionPublisher_group)
        except Exception as e:
            self.get_logger().error(str(e))

    def server_setup(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.settimeout(self.socket_timeout)
        self.client_socket.connect((self.server_ip, self.server_port))
        # Receive welcome message
        self.client_socket.recv(1024)

    def service_setup(self):
        self.move_srv = self.create_service(MoveCscam, '/cscam/move', self.move_cscam, callback_group=self.move_group)
        self.reset_srv = self.create_service(ResetCscam, '/cscam/reset', self.reset_cscam, callback_group=self.resetCscam_group)

    def timer_setup(self):
       # Increase reading frequency for more responsive feedback
       self.reading_servo = self.create_timer(timer_period_sec = 0.05, callback=self.read_position, callback_group=self.reading_servo_group)
    
    def receive_response(self):
        """Optimized method to receive response with timeout handling"""
        try:
            # Set a brief non-blocking attempt first
            self.client_socket.setblocking(0)
            full_response = b""
            start_time = time.time()
            
            # Keep trying until we get data or timeout
            while time.time() - start_time < self.socket_timeout:
                try:
                    chunk = self.client_socket.recv(1024)
                    if chunk:
                        full_response += chunk
                        if len(chunk) < 1024:  # If we got less than buffer size, likely complete
                            break
                except BlockingIOError:
                    # No data yet, try again after a small delay
                    time.sleep(0.01)  # Short sleep to prevent CPU hogging
                    continue
                except socket.timeout:
                    break
                    
            # Reset to blocking mode with timeout
            self.client_socket.setblocking(1)
            self.client_socket.settimeout(self.socket_timeout)
            
            return full_response
        except Exception as e:
            self.get_logger().error(f"Error receiving response: {e}")
            return b""

    def reset_cscam(self, request, response):
        motorid = request.motorid
        if (motorid < 0):
            response.success = False
            response.message = "Cscam motors only go from 1 to 5"
            return response
        
        if (motorid > 5):
            motorid = "all"
            
        # Reduce the lock time on position reading
        self.read_position_flag = False
        time.sleep(0.05)  # Reduced from 0.15s to 0.05s
        
        command = f"reset {motorid}"
        self.get_logger().info(f"Sending command: {command}")
        
        try:
            # Send command
            self.client_socket.send(command.encode())
            if (motorid == 'all'):
                 self.socket_timeout = 30.0
            # Receive response using optimized method
            full_response = self.receive_response()
            
            response.message = full_response.decode() if full_response else "No response received."
            response.success = True if full_response else False
        except Exception as e:
            response.message = f"Error: {e}"
            response.success = False
            
        # Re-enable position reading with reduced delay
        self.read_position_flag = True
        time.sleep(0.05)  # Reduced from 0.15s to 0.05s
        
        return response

    def read_position(self):
        cscam_message = CscamMotors()
        if self.read_position_flag:
            # Send command
            try:
                self.client_socket.send("status".encode())
                
                # Receive response using optimized method
                full_response = self.receive_response()
                
                if full_response:
                    try:
                        full_message = full_response.split()[3:]
                        #self.get_logger().info(full_message[36]) 
                        
                        self.current1  = float(full_message[6])
                        self.current2  = float(full_message[21]) 
                        self.current3  = float(full_message[36])
                        self.current4  = float(full_message[51])
                        
                    except (IndexError, ValueError) as e:
                        self.get_logger().warn(f"Error parsing position data: {e}")
            except Exception as e:
                self.get_logger().warn(f"Error in read_position: {e}")
                
        if (self.current1 is not None) and (self.current2 is not None) and (self.current3 is not None) and (self.current4 is not None):
            cscam_message.lift1position = self.current1
            cscam_message.lift2position = self.current2
            cscam_message.lift3position = self.current3
            cscam_message.lift4position = self.current4
            self.cscamPublisher.publish(cscam_message)

    def move_cscam(self, request, response):
        motorid = request.motorid
        targetPosition = request.position
        
        if (motorid < 0) or (motorid > 5):
            response.success = False
            response.message = "Cscam motors only go from 1 to 5"
            return response
            
        if (targetPosition > 0) or (targetPosition < -80):
            response.success = False
            response.message = "Cscam motors were only coded to run until -480"
            return response
            
        # Reduce the lock time on position reading
        self.read_position_flag = False
        time.sleep(0.05)  # Reduced from 0.15s to 0.05s
        
        command = f"move {motorid} {targetPosition}"
        self.get_logger().info(f"Sending command: {command}")
        
        try:
            # Send command
            self.client_socket.send(command.encode())
            
            # Receive response using optimized method
            full_response = self.receive_response()
            
            response.message = full_response.decode() if full_response else "No response received."
            response.success = True if full_response else False
        except Exception as e:
            response.message = f"Error: {e}"
            response.success = False
            
        # Re-enable position reading with reduced delay
        self.read_position_flag = True
        time.sleep(0.1)  # Reduced from 0.15s to 0.05s
        
        return response

def main():
    rclpy.init()
    cscam_controller = TCPClientNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(cscam_controller, executor=executor)
    cscam_controller.client_socket.close()
    cscam_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()