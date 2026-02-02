import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from std_srvs.srv import Trigger

from gripper_interfaces.srv import ServoAbs
from gripper_interfaces.srv import GripperCom
from gripper_interfaces.srv import PusherMaster

import time  # Import time module for adding delays between movements



#step1_pushers_open: open all pushers
#step2_pushers_close: close all pushers


sleep_time_to_close = 3
sleep_time_dispenser = 3
sleep_time_gripper = 2


class VacuumRobotSequence(Node):
    def __init__(self):
        super().__init__('CookerRobotSequence')
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info(f"cooker robot fully setup")



    def clients_and_callbacks_setup(self):        

        self.callback_pusher1 = MutuallyExclusiveCallbackGroup()
        self.callback_pusher2 = MutuallyExclusiveCallbackGroup()
        self.callback_pusher3 = MutuallyExclusiveCallbackGroup()
        self.callback_pusher4 = MutuallyExclusiveCallbackGroup()

        self.callback_servox = MutuallyExclusiveCallbackGroup()
        self.callback_servoy = MutuallyExclusiveCallbackGroup()
        self.gripper_group = MutuallyExclusiveCallbackGroup()

        self.callback_pushers_group = MutuallyExclusiveCallbackGroup()
        self.dispenser_lifter_group = MutuallyExclusiveCallbackGroup()
        self.gripper_sequence_group = MutuallyExclusiveCallbackGroup()
        self.full_sequence_group = MutuallyExclusiveCallbackGroup()
        self.pusher_loop_group = MutuallyExclusiveCallbackGroup()

        self.pusher1_client = self.create_client(PusherMaster, "/pusher/one/open_close", callback_group=self.callback_pusher1)
        while not self.pusher1_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/pusher/one/open_close not available, waiting again...')

        self.pusher2_client = self.create_client(PusherMaster, "/pusher/two/open_close", callback_group=self.callback_pusher2)
        while not self.pusher2_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/pusher/two/open_close not available, waiting again...')

        self.pusher3_client = self.create_client(PusherMaster, "/pusher/three/open_close", callback_group=self.callback_pusher3)
        while not self.pusher3_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/pusher/three/open_close not available, waiting again...')
        
        self.pusher4_client = self.create_client(PusherMaster, "/pusher/four/open_close", callback_group=self.callback_pusher4)
        while not self.pusher4_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/pusher/four/open_close not available, waiting again...')

        self.servo_x_client = self.create_client(ServoAbs, "/servo_controllerX/moveabs", callback_group=self.callback_servox)
        while not self.servo_x_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerX/moveabs not available, waiting again...')

        self.servo_y_client = self.create_client(ServoAbs, "/servo_controllerY/moveabs", callback_group=self.callback_servoy)
        while not self.servo_y_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerY/moveabs not available, waiting again...')


        self.gripper_client = self.create_client(GripperCom, "/gripper_controller/open_close", callback_group=self.gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/gripper_controller/open_close not available, waiting again...')



    def service_setup(self):
        self.sequence_pushers_srv = self.create_service(Trigger,'/cooker_robot/cooker_sequence_pushers',self.sequence_pushers, callback_group=self.callback_pushers_group)
        self.dispenser_lifter_srv = self.create_service(Trigger,'/cooker_robot/dispenser_lifter_sequence', self.sequence_dispenser_lifter, callback_group=self.dispenser_lifter_group)
        self.gripper_sequence_srv = self.create_service(Trigger, '/cooker_robot/gripper_sequence', self.gripper_sequence, callback_group=self.gripper_sequence_group)
        self.full_sequence_srv = self.create_service(Trigger, '/cooker_robot/full_sequence', self.full_sequence, callback_group=self.full_sequence_group)
        self.pusher_loop_srv = self.create_service(Trigger, '/cooker_robot/pusher_loop', self.pusher_loop, callback_group=self.pusher_loop_group)
    
    def pusher_loop(self, request, response):
        while True:
            pusher1Request = PusherMaster.Request()
            pusher1Request.command = 'open'
            pusher1Request.pusher_id = 1
            response_open = self.pusher1_client.call(pusher1Request)
            time.sleep(7)
            pusher1Request = PusherMaster.Request()
            pusher1Request.command = 'close'
            pusher1Request.pusher_id = 1
            response_close = self.pusher1_client.call(pusher1Request)
        response.success = True
        return response

    def step1_pushers_open(self):
        pusher1Request = PusherMaster.Request()
        pusher1Request.pusher_id = 1
        pusher1Request.command = "open"

        pusher2Request = PusherMaster.Request()
        pusher2Request.pusher_id = 2
        pusher2Request.command = "open"
        
        pusher3Request = PusherMaster.Request()
        pusher3Request.pusher_id = 3
        pusher3Request.command = "open"

        pusher4Request = PusherMaster.Request()
        pusher4Request.pusher_id = 4
        pusher4Request.command = "open"

        
        response1 = self.pusher1_client.call(pusher1Request)
        if response1.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher1")
        else:
            self.get_logger().info(f"Signal to pusher1 was sent")
        
        response2 = self.pusher2_client.call(pusher2Request)
        if response2.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher2")
        else:
            self.get_logger().info(f"Signal to pusher2 was sent")

        response3 = self.pusher3_client.call(pusher3Request)
        if response3.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher3")
        else:
            self.get_logger().info(f"Signal to pusher3 was sent")

        response4 = self.pusher4_client.call(pusher4Request)
        if response4.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher4")
        else:
            self.get_logger().info(f"Signal to pusher4 was sent")
        
        finalResponse = response1.success and response2.success and response3.success and response4.success
        self.get_logger().info(f"The finalresponse is {finalResponse}")
        
        if finalResponse:
            self.get_logger().info(f"all pusher signals to open were sent")
        else:
            self.get_logger().info(f"A pusher was not open")

        return finalResponse
    
    def step2_pushers_close(self):
        pusher1Request = PusherMaster.Request()
        pusher1Request.pusher_id = 1
        pusher1Request.command = "close"

        pusher2Request = PusherMaster.Request()
        pusher2Request.pusher_id = 2
        pusher2Request.command = "close"
        
        pusher3Request = PusherMaster.Request()
        pusher3Request.pusher_id = 3
        pusher3Request.command = "close"

        pusher4Request = PusherMaster.Request()
        pusher4Request.pusher_id = 4
        pusher4Request.command = "close"

        
        response1 = self.pusher1_client.call(pusher1Request)
        if response1.success == False:
            self.get_logger().info(f"Could not sent close signal to pusher1")
        else:
            self.get_logger().info(f"Signal to pusher1 was sent")
        
        response2 = self.pusher2_client.call(pusher2Request)
        if response2.success == False:
            self.get_logger().info(f"Could not sent close signal to pusher2")
        else:
            self.get_logger().info(f"Signal to pusher2 was sent")

        response3 = self.pusher3_client.call(pusher3Request)
        if response3.success == False:
            self.get_logger().info(f"Could not sent close signal to pusher3")
        else:
            self.get_logger().info(f"Signal to pusher3 was sent")

        response4 = self.pusher4_client.call(pusher4Request)
        if response4.success == False:
            self.get_logger().info(f"Could not sent close signal to pusher4")
        else:
            self.get_logger().info(f"Signal to pusher4 was sent")
        
        finalResponse = response1.success and response2.success and response3.success and response4.success
        self.get_logger().info(f"The finalresponse is {finalResponse}")
        
        if finalResponse:
            self.get_logger().info(f"all pusher signals to close were sent")
        else:
            self.get_logger().info(f"A pusher was not close")

        return finalResponse

    def step2_move_servox(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = 500.0
        servoxRequest.velocity = 250.0
        response = self.servo_x_client.call(servoxRequest)
        return response.success

    def step1_move_servoy(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = -20.0
        servoxRequest.velocity = 600.0
        response = self.servo_y_client.call(servoxRequest)
        return response.success
    
    def step3_go_home(self):
        servoyRequest = ServoAbs.Request()
        servoyRequest.position = -1.0
        servoyRequest.velocity = 150.0
        response = self.servo_y_client.call(servoyRequest)
      
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = 1.0
        servoxRequest.velocity = 150.0
        response = self.servo_x_client.call(servoxRequest)
       
        return response.success

    def step1_close_gripper(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'close'
        response = self.gripper_client.call(gripperRequest)
        return response.success

    def step2_open_gripper(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'open'
        response = self.gripper_client.call(gripperRequest)
        
        return response.success

    def gripper_sequence(self, request, response):
        response.success = False
        for _ in range(3):
            step1_response = self.step1_close_gripper()
            if step1_response != True:
                self.get_logger().info(f"The sequence failed, first step was not completed")
                return response
            self.get_logger().info(f"Step1 completed")
            # Add a small pause to ensure the robot stops before moving to the next position
            time.sleep(sleep_time_gripper)  


            step2_response = self.step2_open_gripper()
            if step2_response != True:
                self.get_logger().info(f"The sequence failed, second step was not completed")
                return response
            self.get_logger().info(f"Step2 completed")
        
        response.success = True
        self.get_logger().info(f"Sequence gripper completed")
        
        return response
         
    def sequence_pushers(self,request, response):
        
        response.success = False

        ##Open and close 4 pushers
        #########################################################################

        step1_response = self.step1_pushers_open()
        if step1_response != True:
            self.get_logger().info(f"The sequence failed, first step was not completed")
            return response
        self.get_logger().info(f"Step1 completed")
        # Add a small pause to ensure the robot stops before moving to the next position
        time.sleep(sleep_time_to_close)  


        step2_response = self.step2_pushers_close()
        if step2_response != True:
            self.get_logger().info(f"The sequence failed, second step was not completed")
            return response
        self.get_logger().info(f"Step2 completed")
         
        #################################################################################


        response.success = True
        self.get_logger().info(f"Sequence pushers completed")
        return response
    
    def sequence_dispenser_lifter(self, request, response):
        response.success = False
        step1_response = self.step1_move_servoy()
        if step1_response != True:
            self.get_logger().info(f"The sequence failed, first step was not completed")
            return response
        self.get_logger().info(f"Step1 completed")
        # Add a small pause to ensure the robot stops before moving to the next position
         
        time.sleep(sleep_time_dispenser) 
        step2_response = self.step2_move_servox()
        if step2_response != True:
            self.get_logger().info(f"The sequence failed, second step was not completed")
            return response
        self.get_logger().info(f"Step2 completed")

        time.sleep(sleep_time_dispenser) 
        step3_response = self.step3_go_home()
        if step3_response != True:
            self.get_logger().info(f"The sequence failed, third step was not completed")
            return response
        self.get_logger().info(f"Step3 completed")

        response.success = True
        self.get_logger().info(f"Sequence dispenser lifter completed")
        return response
    
    def step0_move_servox(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = 650.0
        servoxRequest.velocity = 500.0
        response = self.servo_x_client.call(servoxRequest)
        return response.success
    
    def step2_open_pusher1(self):

        pusher1Request = PusherMaster.Request()
        pusher1Request.pusher_id = 1
        pusher1Request.command = "open"

        response1 = self.pusher1_client.call(pusher1Request)
        if response1.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher1")
        else:
            self.get_logger().info(f"Signal to pusher1 was sent")
        return response1.success

    def step3_close_pusher1(self):
        pusher1Request = PusherMaster.Request()
        pusher1Request.pusher_id = 1
        pusher1Request.command = "close"

        response1 = self.pusher1_client.call(pusher1Request)
        if response1.success == False:
            self.get_logger().info(f"Could not sent open signal to pusher1")
        else:
            self.get_logger().info(f"Signal to pusher1 was sent")
        time.sleep(5)
        return response1.success

    def step4_move_servox(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = 400
        servoxRequest.velocity = 500.0
        response = self.servo_x_client.call(servoxRequest)
        return response.success
    
    def step5_close_gripper(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'close'
        response = self.gripper_client.call(gripperRequest)
        return response.success

    def step6_move_servox(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = -100.0
        servoxRequest.velocity = 500.0
        response = self.servo_x_client.call(servoxRequest)
        return response.success
    
    def step7_move_servoy(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = -720.0
        servoxRequest.velocity = 500.0
        response = self.servo_y_client.call(servoxRequest)
        return response.success
    
    def step8_move_servoy(self):
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = -20.0
        servoxRequest.velocity = 500.0
        response = self.servo_y_client.call(servoxRequest)
        return response.success

    def step8_open_gripper(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'open'
        response = self.gripper_client.call(gripperRequest)
        return response.success
    
    def step9_go_home(self):
        servoyRequest = ServoAbs.Request()
        servoyRequest.position = 0.0
        servoyRequest.velocity = 100.0
        response = self.servo_y_client.call(servoyRequest)
      
        servoxRequest = ServoAbs.Request()
        servoxRequest.position = 0.0
        servoxRequest.velocity = 100.0
        response = self.servo_x_client.call(servoxRequest)
       
        return response.success

    def full_sequence(self, request, response):
        response.success = False
        while True:

            step1_response = self.step1_move_servoy()
            if step1_response != True:
                self.get_logger().info(f"The sequence failed, first step was not completed")
                return response
            self.get_logger().info(f"Step1 completed")

            step0_response = self.step0_move_servox()
            if step0_response != True:
                self.get_logger().info(f"The sequence failed, zero step was not completed")
                return response
            self.get_logger().info(f"Step0 completed")

          
           

            

            # step2_response = self.step2_open_pusher1()
            # if step2_response != True:
            #     self.get_logger().info(f"The sequence failed, second step was not completed")
            #     return response
            # self.get_logger().info(f"Step2 completed")
            # time.sleep(12)

            # step3_response = self.step3_close_pusher1()
            # if step3_response != True:
            #     self.get_logger().info(f"The sequence failed, third step was not completed")
            #     return response
            # self.get_logger().info(f"Step3 completed")
            # time.sleep(12)
            # step4_response = self.step4_move_servox()
            # if step4_response != True:
            #     self.get_logger().info(f"The sequence failed, fourth step was not completed")
            #     return response
            # self.get_logger().info(f"Step4 completed")
            # time.sleep(2)

            # step5_response = self.step5_close_gripper()
            # if step5_response != True:
            #     self.get_logger().info(f"The sequence failed, fifth step was not completed")
            #     return response
            # self.get_logger().info(f"Step5 completed")
            # time.sleep(1)

            # step6_response = self.step6_move_servox()
            # if step6_response != True:
            #     self.get_logger().info(f"The sequence failed, sixth step was not completed")
            #     return response
            # self.get_logger().info(f"Step6 completed")
            # time.sleep(1)

            step7_response = self.step7_move_servoy()
            if step7_response != True:
                self.get_logger().info(f"The sequence failed, seventh step was not completed")
                return response
            self.get_logger().info(f"Step7 completed")

            time.sleep(7)

            step8_response = self.step8_move_servoy()
            if step8_response != True:
                self.get_logger().info(f"The sequence failed, seventh step was not completed")
                return response
            self.get_logger().info(f"Step7 completed")






            step6_response = self.step6_move_servox()
            if step6_response != True:
                self.get_logger().info(f"The sequence failed, sixth step was not completed")
                return response
            self.get_logger().info(f"Step6 completed")
            time.sleep(1)


            # step8_response = self.step8_open_gripper()
            # if step8_response != True:
            #     self.get_logger().info(f"The sequence failed, eightth step was not completed")
            #     return response
            # self.get_logger().info(f"Step8 completed")
            # time.sleep(1)

            # step9_response = self.step9_go_home()
            # if step9_response != True:
            #     self.get_logger().info(f"The sequence failed, ninth step was not completed")
            #     return response
            # self.get_logger().info(f"Step9 completed")

            self.get_logger().info(f"Full sequence completed")





        # Add a small pause to ensure the robot stops before moving to the next position
        response.success = True
        self.get_logger().info(f"Full sequence completed")
        return response
    

def main():
    rclpy.init()
    vacuum_robot_sequence = VacuumRobotSequence()
    executor = MultiThreadedExecutor()
    rclpy.spin(vacuum_robot_sequence, executor=executor)
    vacuum_robot_sequence.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  