import serial
import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
import numpy as np
import time
import sys

from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from gripper_interfaces.srv import ServoAbs
from gripper_interfaces.srv import ServoAbsNice
from gripper_interfaces.msg import ServoPosition
from gripper_interfaces.srv import GripperCom
from std_srvs.srv import Trigger

COMMANDS ={"read_position": "PFB\r", "read_velocity":"MVEL\r", "disable_command":"K\r", 
           "software_enable_command":"en\r\n", "move_servo_absolute_value":"moveabs", 
           "get_stop_signal":"stopped\r", "set_acceleration": "acc", "set_deceleration":"dec",
           "home_command":"homecmd\r", "home_state_signal":"homestate\r", "unlocking_signal":"\\1\r", "check_status":"active\r"}

class ServoMotorNode(Node):
    def __init__(self):
        super().__init__('servo_motor_controller')
        self.name = "X"
        self.tty = "/dev/GantryUSB" + self.name
        #self.tty = "/dev/ttyACM1"
        self.read_write_feedback = True
        self.current_position  = None
        #self.limit_position = 1950  #mm
        #self.limit_velocity = 500 #mm/s
        self.connect_serial()
        self.call_back_groups_setup()
        self.timer_setup()
        self.service_setup()
        self.publisher_setup()
        self.get_logger().info(f"Servo motor {self.name} fully setup!!\n")

    def limits(self):
        if self.name == "Y":
            self.limits = 783.587
        elif self.name == "X":
            self.limits = 1100
        else:
            self.get_logger().info("ERROR: the naming of the servo is wrong")
    
    def compute_vel_acc_decc(self, distance, time):
        speed = abs(distance) / time
        acc = speed/(0.1*time)
        decc = speed/(0.9*time)
        return (speed ,acc, decc)
        
    

    def publisher_setup(self):
        try:
            self.servoPositionPublisher = self.create_publisher(ServoPosition, f"/servo_controller{self.name}/position" ,10,callback_group=self.servoPositionPublisher_group)
        except Exception as e:
            self.get_logger().error(str(e))

    def call_back_groups_setup(self):
        self.servoPositionPublisher_group = MutuallyExclusiveCallbackGroup()
        self.reading_servo_group = MutuallyExclusiveCallbackGroup()
        self.servoControllerMoveabs_group = MutuallyExclusiveCallbackGroup()
        self.servoHomingProcess_group = MutuallyExclusiveCallbackGroup()
        self.servoNiceSpeed_group = MutuallyExclusiveCallbackGroup()
    
    def service_setup(self):
        self.move_servo = self.create_service(ServoAbs,f"/servo_controller{self.name}/moveabs",self.moveabs_servo,callback_group=self.servoControllerMoveabs_group)
        self.homing_process_service = self.create_service(Trigger, f"/servo_controller{self.name}/homing",self.homing_process ,callback_group=self.servoHomingProcess_group)
        self.servoNiceSpeed_service = self.create_service(ServoAbsNice, f"/servo_controller{self.name}/moveabsNice", self.moveabs_servo_nice, callback_group=self.servoNiceSpeed_group)
       

    
    def moveabs_servo_nice(self, request, response):
        try:
            if request.time < 0.5:
                response.success = False
                response.message = "The time is too low, increase to at least 0.5s"
                return response
            if self.current_position != None:
                distance = request.position - self.current_position
                speed , acc, dec = self.compute_vel_acc_decc(distance, request.time)

                self.read_write_feedback = False
                if self.check_connection():
                    command_acceleration = f"{COMMANDS['set_acceleration']} {round(acc,3)}\r"
                    time.sleep(0.1)
                    self.ser.write(command_acceleration.encode()) # configure acceleration
                else:
                  response.success = False
                  response.message = f"Communication with servo motor {self.name} got interrupted"
                  return response

                if self.check_connection():
                    command_deceleration = f"{COMMANDS['set_deceleration']} {round(dec,3)}\r"
                    time.sleep(0.1)
                    self.ser.write(command_deceleration.encode()) # configure deceleration
                else:
                  response.success = False
                  response.message = f"Communication with servo motor {self.name} got interrupted"
                  return response

                if self.check_connection():
                    command_moveabs = f"{COMMANDS['move_servo_absolute_value']} {request.position} {round(speed,3)}\r"
                    time.sleep(0.1)
                    self.ser.write(command_moveabs.encode()) # send moveabs command
                else:
                  response.success = False
                  response.message = f"Communication with servo motor {self.name} got interrupted"
                  return response

                stop_signal = 0
                while stop_signal < 1:
                    stop_signal = self.detect_stop_signal()
                    if stop_signal < 0:
                        response.success = False
                        response.message = f"The servo motor {self.name} made an EMERGENCY STOP!!!"
                        return response
                self.read_write_feedback = True
                time.sleep(0.1)
                if ((abs(self.current_position - request.position)/100) < 0.9) and (stop_signal > 0):
                    response.success = True
                    response.message = f"The servo motor {self.name} arrived the desired position"
                else:
                    response.success = False
                    response.message = f"The servo motor {self.name} received a safe stop signal (1 or 2), but the position was not reached"
            else:
                response.success = False
                response.message = f"The servo motor {self.name} did not finish initializing...try again"
            
            
            return response
        except Exception as e:
            self.get_logger().error(str(e)) 





         
    # def moveabs_servo(self, request, response):
    #     try:
    #         #Wait to turn of response from feedback
    #         self.read_write_feedback = False
    #         time.sleep(0.1)

    #         # Send command to move servo
    #         position = request.position             #mm
    #         velocity = request.velocity             #mm/s
    #         if position > self.limit_position:
    #             response.success = False
    #             response.message = f"The position exceeds {self.limit_position}, reduce distance in mm"
    #             return response
    #         if velocity > self.limit_velocity:
    #            response.success = False
    #            response.message = f"The position exceeds {self.limit_velocity}, reduce distance in mm"
    #            return response
    #         command = f"{COMMANDS['move_servo_absolute_value']} {position} {velocity}\r"
    #         if self.check_connection():
    #             self.ser.write(command.encode())
    #             time.sleep(0.1)
    #             self.read_write_feedback = True
    #             time.sleep(0.1)
    #             while (abs(self.current_position - position)/100) > 0.5:
    #                 self.get_logger().info("Servo is still moving....please wait......")
    #             response.success = True
    #             response.message = "Command to move was sent successfully"
    #         else:
    #             response.success = False
    #             response.message = "It was not possible to connect to the servo"
    #         return response
    #     except Exception as e:
    #         self.get_logger().error(str(e))

    def moveabs_servo(self, request, response):
        try:
            #* Wait to turn of response from feedback
            self.read_write_feedback = False
            time.sleep(0.1)
         
            position = request.position             #* mm
            velocity = request.velocity             #* mm/s
            command = f"{COMMANDS['move_servo_absolute_value']} {position} {velocity}\r"
           
            if self.check_connection():
                #* Sending command to move servo
                self.ser.write(command.encode())
                time.sleep(0.1)

                stop_signal = 0
                while stop_signal < 1:
                    stop_signal = self.detect_stop_signal()
                    if stop_signal < 0:
                        response.success = False
                        response.message = f"The servo motor {self.name} made an EMERGENCY STOP!!!"
                        return response
                self.read_write_feedback = True
                time.sleep(0.1)

                if ((abs(self.current_position - position)/100) < 0.9) and (stop_signal > 0):
                    response.success = True
                    response.message = f"The servo motor {self.name} arrived the desired position"
                else:
                    response.success = False
                    response.message = f"The servo motor {self.name} received a safe stop signal (1 or 2), but the position was not reached"
            else:
                response.success = False
                response.message = f"Communication with servo motor {self.name} got interrupted"
            return response
        except Exception as e:
            self.get_logger().error(str(e))


    def homing_process(self, request, response):
        self.get_logger().info(f"Homming process {self.name} triggered")
        sys.stdout.flush()
        home_command = COMMANDS["home_command"]
        self.read_write_feedback = False
        time.sleep(0.1)
        if self.check_connection():
            self.ser.write(home_command.encode())
            response.success = True
            response.message = "homing signal was sent"
        else:
             response.success = False
             response.message = f"Communication with servo motor {self.name} got interrupted"
        self.read_write_feedback = True
        sys.stdout.flush()
        return response





    def read_position(self):
        servo_position = ServoPosition()
        if self.read_write_feedback:
            if self.check_connection():
                command = COMMANDS["read_position"]
                self.ser.write(command.encode())
                time.sleep(0.1)
                response = self.ser.read(self.ser.in_waiting or 1).decode()
                #self.get_logger().info(f"Complete response {response}")
                response = response.strip().split("\r\n")[1].split(" ")[0]
                
                try:
                    response_float = float(response)
                    self.current_position = response_float

                    #self.get_logger().info(f"current position: \n{self.current_position}\n")
                    # self.get_logger().info(f"total of {len(response)} responses \n")
                except:
                    pass
                    

            else:
                self.get_logger().info("Communication with the servo got lost")
        if self.current_position is not None:
            servo_position.position = self.current_position
            self.servoPositionPublisher.publish(servo_position)
        
    def timer_setup(self):
        self.reading_servo = self.create_timer(timer_period_sec=0.1,callback=self.read_position, callback_group=self.reading_servo_group)

    def connect_serial(self):
        try: 
            self.ser = serial.Serial(port=self.tty, baudrate= 115200, timeout = 1)
            if self.ser.is_open:
                try:
                    self.ser.write(COMMANDS["unlocking_signal"].encode())
                    time.sleep(0.5)
                    
                except:
                    self.get_logger().error(f"The {self.tty} could not be unlocked")
                    return False
                self.get_logger().info(f"The {self.tty} Port is Opened Now!")
                self.ser.write(COMMANDS["software_enable_command"].encode())
                # time.sleep(0.5)
                # active = 0
                # while 0 == active:
                #     time.sleep(0.1)
                #     self.ser.write(COMMANDS["check_status"].encode())
                #     time.sleep(0.5)
                #     active_signal_complete = self.ser.read(self.ser.in_waiting or 1).decode().strip().split("\r\n")
                #     self.get_logger().info(f"Active signal complete {active_signal_complete}")
                #     if [''] == active_signal_complete:
                #         self.get_logger().info("TURNING OFF!!!")                   
                #         time.sleep(0.1)
                #         self.ser.write(COMMANDS["disable_command"].encode())
                #         time.sleep(0.5)
                #         self.ser.write(COMMANDS["software_enable_command"].encode())
                #         time.sleep(0.1)
                #         active_signal_complete2 = self.ser.read(self.ser.in_waiting or 1).decode().strip().split("\r\n")
                #         self.get_logger().info(f"Active signal complete2 {active_signal_complete2}")



                
            else:
                self.get_logger().info(f"The {self.tty} Port is Opened by another program")
        except Exception as e:
            self.get_logger().error(e)
            return False
   
    
    def check_connection(self):
        try:
            if self.ser.is_open:
                return True
            else:
                return False
        except:
            return False
    
    def detect_stop_home_signal(self):
        time.sleep(0.1)
        self.ser.write(COMMANDS['home_state_signal'].encode())
        
        time.sleep(0.1)
        stop_home_signal_complete = self.ser.read(self.ser.in_waiting or 1).decode().strip().split("\r\n")
        print(f"stop_signal_complete was: {stop_home_signal_complete}")
        stop_home_signal = stop_home_signal_complete[-2]
        print(f"stop_signal was: {stop_home_signal}")
        
        try:
            stop_home_signal = int(stop_home_signal)
        except Exception as e:
            self.get_logger().info(str(e))
            self.get_logger().info(f'the stop signal modified was {stop_home_signal}') 
        return stop_home_signal

        



    def detect_stop_signal(self):
        
        #?Wait to turn of response from feedback
        self.read_write_feedback = False
        time.sleep(0.1)

        self.ser.write(COMMANDS['get_stop_signal'].encode())
        time.sleep(0.1)
        stop_signal_complete = self.ser.read(self.ser.in_waiting or 1).decode().strip().split("\r\n")
        print(f"stop_signal_complete was: {stop_signal_complete}")
        
        stop_signal = stop_signal_complete[-2]
        print(f"stop_signal was: {stop_signal}")


        try: 
            #?Convert the stop signal to integer
            stop_signal = int(stop_signal)
        except Exception as e:
            self.get_logger().info(str(e))
            self.get_logger().info(f'the stop signal modified was {stop_signal}') 

        self.read_write_feedback = True
        time.sleep(0.1)

        return stop_signal

    def move_servo_signal(self, position, velocity):

        self.read_write_feedback = False
        time.sleep(0.1)

        command = f"{COMMANDS['move_servo_absolute_value']} {position} {velocity}\r"
        self.ser.write(command.encode())
        
        time.sleep(0.1)
        self.read_write_feedback = True







def main():
    rclpy.init()
    servoMotor = ServoMotorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(servoMotor, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        servoMotor.ser.write(COMMANDS["disable_command"].encode())
        servoMotor.get_logger().error("ErrorServoMotorX")
        servoMotor.destroy_node()
       

if __name__ == '__main__':
    main()    