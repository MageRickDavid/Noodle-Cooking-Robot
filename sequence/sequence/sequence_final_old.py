import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import subprocess

from std_srvs.srv import Trigger

from gripper_interfaces.srv import ServoAbs
from gripper_interfaces.srv import GripperCom
from gripper_interfaces.srv import PusherMaster
from gripper_interfaces.srv import ServoAbsNice
from gripper_interfaces.srv import CartesianLocation
from gripper_interfaces.srv import RotatingServo
from gripper_interfaces.srv import ZaxisCommands
from gripper_interfaces.srv import RotatingServoTimeBased
from gripper_interfaces.srv import PumpControl
from gripper_interfaces.srv import ServingNoodle
from gripper_interfaces.srv import PusherTimeMaster
from gripper_interfaces.srv import GrabExitBucket
from gripper_interfaces.srv import MoveCscam



import time  # Import time module for adding delays between movements
import subprocess


trajectory_time = 2.0
locations = {
            "reset": (0.0, 0.0),
             "home": (0.0,0.0),

            "head1":(859.26, 719.883),
            "head1_back": (686.532, 719.883),

            "head2":(859.26, 536.183),
            "head2_back": (686.532, 536.529),

            "head3":(859.26, 229.834),
            "head3_back":(686.532, 229.834),


            "head4":(859.26, 42.762),
            "head4_back": (686.532, 42.762),

            "inlet0": (686.532, 720.0),
                                    
            "inlet1": (930.000,730.917),
            "inlet2": (930.000, 730.963),

}

angles = {
    "rotating_z_reset": 180.0,
    "rotating_z_drop": 270.0,

    "rotating_x_reset": 270.0,
    "rotating_x_drop": 90.0,

    "inlet1x": 200.0,
    "inlet2x": 110.0,
    "inlet3x": 110.0,
}

lifter_location = {
    "lift_up":-80.0
}

class CookerRobotSequence(Node):
    def __init__(self):
        super().__init__('CookerRobotSequenceAlgorithm')
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info(f"cooker robot fully setup")

    def clients_and_callbacks_setup(self):

        #*Linear Servodriver
        self.callback_servox = MutuallyExclusiveCallbackGroup()
        self.callback_servoy = MutuallyExclusiveCallbackGroup()
        self.callback_servox_homing = MutuallyExclusiveCallbackGroup()
        self.callback_servoy_homing = MutuallyExclusiveCallbackGroup()
       
        #*Rotating Servodriver
        self.callback_rotating_servoZ_time = MutuallyExclusiveCallbackGroup()
        self.callback_rotating_servoX_time = MutuallyExclusiveCallbackGroup()
        
        #* Gripper
        self.callback_gripper_group = MutuallyExclusiveCallbackGroup()
        
        #* Pusher
        self.callback_pusher_master_group = MutuallyExclusiveCallbackGroup()

        #* CSCAM motors
        self.callback_cscam_master_group = MutuallyExclusiveCallbackGroup()
       

        #*Actions......
        self.dispense_noodle_group = MutuallyExclusiveCallbackGroup()
        self.cook_noodle_group = MutuallyExclusiveCallbackGroup()

        #*Linear Servodriver Client
        self.servo_x_client = self.create_client(ServoAbsNice, "/servo_controllerX/moveabsNice", callback_group=self.callback_servox)
        while not self.servo_x_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerX/moveabsNice not available, waiting again...')

        self.servo_y_client = self.create_client(ServoAbsNice, "/servo_controllerY/moveabsNice", callback_group=self.callback_servoy)
        while not self.servo_y_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/servo_controllerY/moveabsNice not available, waiting again...')
        
        #*Rotating Servodriver Client
        self.rotating_servoZ_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoZ/rotatingTimeBased", callback_group=self.callback_rotating_servoZ_time)
        while not self.rotating_servoZ_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoZ/rotatingTimeBased not available, waiting again...')

        self.rotating_servoX_time_based_client = self.create_client(RotatingServoTimeBased, "/rotating_servoX/rotatingTimeBased", callback_group=self.callback_rotating_servoX_time)
        while not self.rotating_servoX_time_based_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/rotating_servoX/rotatingTimeBased not available, waiting again...')

        #* Gripper Client
        self.gripper_client = self.create_client(GripperCom, "/gripper_controller/open_close", callback_group=self.callback_gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
             self.get_logger().info('/gripper_controller/open_close not available, waiting again...')

        #* Pusher Client
        self.pusher_client_master = self.create_client(PusherTimeMaster, "/pusher/open_close", callback_group=self.callback_pusher_master_group)
        while not self.pusher_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/pusher/open_close not available, waiting again...')

        #*Cscam Client
        self.cscam_client_master = self.create_client(MoveCscam, "/cscam/move", callback_group=self.callback_cscam_master_group)
        while not self.cscam_client_master.wait_for_service(timeout_sec=20.0):
            self.get_logger().info('/cscam/move not available, waiting again...')
        
        
        # self.pump_client = self.create_client(PumpControl, "/pump_control", callback_group= self.callback_pump_controller_group)
        # while not self.pump_client.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('/pump_control not available, waiting again...')
    
    def service_setup(self):
        self.dispense_noodle_srv        =  self.create_service(ServingNoodle, "/Dispense", self.dispense_noodle, callback_group=self.dispense_noodle_group)
        self.cook_noodle_srv            =  self.create_service(ServingNoodle, "/Cook"   , self.cook_noodle,     callback_group=self.cook_noodle_group)

    def pusher_master(self, idPusher, command, timeToComplete = -1):
        pusherMasterRequest = PusherTimeMaster.Request()
        pusherMasterRequest.id = idPusher
        pusherMasterRequest.command = command
        self.get_logger().info(f"TIME TO COMPLETE SELECTED {timeToComplete}")
        pusherMasterRequest.time = timeToComplete
        pusherMasterResponse = self.pusher_client_master.call(pusherMasterRequest)
        return pusherMasterResponse.success
    
    def lifter_master_position(self, lifterid, position):
        if lifterid > 4:
            return False
        cscamMasterRequest = MoveCscam.Request()
        cscamMasterRequest.motorid = lifterid + 1
        cscamMasterRequest.position = position
        cscamMasterResponse = self.cscam_client_master.call(cscamMasterRequest)
        return cscamMasterResponse.success



    async def cartesian_location(self, x, y, time=2.0): #? Default time is two seconds
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        futurex = self.servo_x_client.call_async(xLocationRequest)
        futurey = self.servo_y_client.call_async(yLocationRequest)

        resultx = await futurex
        resulty = await futurey
        if (resulty.success and resultx.success):
            return True
        else:
            return False
    
    async def rotating_translatingZ_movement(self, x, y, angle, time):
        xLocationRequest = ServoAbsNice.Request()
        xLocationRequest.position = x
        xLocationRequest.time = time
        
        yLocationRequest = ServoAbsNice.Request()
        yLocationRequest.position = y
        yLocationRequest.time = time

        angleRequest = RotatingServoTimeBased.Request()
        angleRequest.angle = angle
        angleRequest.time = time

        futurex         = self.servo_x_client.call_async(xLocationRequest)
        futurey         = self.servo_y_client.call_async(yLocationRequest)
        futureAngleZ    = self.rotating_servoZ_time_based_client.call_async(angleRequest)

        resultx = await futurex
        resulty = await futurey
        resultAngleZ = await futureAngleZ
        if (resulty.success and resultx.success and resultAngleZ.success):
            return True
        else:
            return False
    

    def rotate_servoZ(self, angle):
        rotatingServoTimeBasedRequest = RotatingServoTimeBased.Request()
        rotatingServoTimeBasedRequest.angle = angle
        rotatingServoTimeBasedRequest.time = 2.0
        response = self.rotating_servoZ_time_based_client.call(rotatingServoTimeBasedRequest)
        return response.success
    
    def rotate_servoX(self, angle):
        rotatingServoTimeBasedRequest = RotatingServoTimeBased.Request()
        rotatingServoTimeBasedRequest.angle = angle
        rotatingServoTimeBasedRequest.time = 2.0
        response = self.rotating_servoX_time_based_client.call(rotatingServoTimeBasedRequest)
        return response.success

    def gripper_close(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'close'
        response = self.gripper_client.call(gripperRequest)
        return response.success
    
    def gripper_open(self):
        gripperRequest = GripperCom.Request()
        gripperRequest.message = 'open'
        response = self.gripper_client.call(gripperRequest)
        return response.success

    async def grab_to_exit_master(self, head):
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result

        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        
        #? This part can change, I think the lifters will be already up by this point!!!!!
        # try:
        #     send_http_request(http_commands[f'lifter_{head}_up'])
        #     self.get_logger().info(f"Waiting for lifter {head} to go up")
        # except Exception as e:
        #     self.get_logger().error(f"lifter {head} failed while going up")
        #     self.get_logger().error(f"lifter {head} says: {str(e)}")
        #     return False
        # time.sleep(7)
        #?###########################################################

        #send_http_request(http_commands[f'lifter_{head}_up'])
        
        time.sleep(5)
        
        self.get_logger().info(f"Closing gripper!!!")
        gripperResponse = self.gripper_close()
        if gripperResponse != True:
            self.get_logger().error("something failed when closing the gripper")
            return gripperResponse
        
        # time.sleep(5)

        #send_http_request(http_commands['z_up'])
#
        #send_http_request(http_commands[f'lifter_{head}_down'])
        time.sleep(5)

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        # self.get_logger().info(f"Z-Clearance")
        # location_result = await self.cartesian_location(*locations[f"z_clearance"], time=trajectory_time)
        # if  location_result != True:
        #     self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
        #     return location_result
        #send_http_request(http_commands[f'z_clearance'])
        time.sleep(6)

        self.get_logger().info(f"rotating servoZ to drop position")
        rotatingServoZResponse = self.rotate_servoZ(angle=angles["rotating_z_drop"]) 
        if rotatingServoZResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to drop position")
            return rotatingServoZResponse

        self.get_logger().info(f"Approaching to inlet0")
        location_result = await self.cartesian_location(*locations["inlet0"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet1 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet1x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet1 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to inlet1")
        location_result = await self.cartesian_location(*locations["inlet1"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to inlet1")
            return location_result

        self.get_logger().info(f"rotating servoX to inlet2 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet2x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet2 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to inlet2")
        location_result = await self.cartesian_location(*locations["inlet2"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching inlet2")
            return location_result
        
        self.get_logger().info(f"rotating servoX to inlet3 position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["inlet3x"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to inlet3 position")
            return rotatingServoXResponse

        self.get_logger().info(f"Approaching to head_2_back")
        location_result = await self.cartesian_location(*locations[f"head2_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head 2 to exit failed when approaching to head_2_back")
            return location_result
        
        self.get_logger().info(f"rotating servoX to reset position")
        rotatingServoXResponse = self.rotate_servoX(angle=angles["rotating_x_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoX, to reset position")
            return rotatingServoXResponse
        
        self.get_logger().info(f"rotating servoZ to reset position")
        rotatingServoXResponse = self.rotate_servoZ(angle=angles["rotating_z_reset"]) 
        if rotatingServoXResponse != True:
            self.get_logger().error("Something failed when rotating the servoZ, to reset position")
            return rotatingServoXResponse
        
        # self.get_logger().info(f"Approaching to head_{head}_back and rotating servoZ to reset position")
        # movement_result = await self.rotating_translatingZ_movement(*locations[f"head{head}_back"],angle=angles["rotating_z_reset"], time=2.0)
        # if movement_result != True:
        #     self.get_logger().error("Something failed when rotating and translating at the same time")
        #     return movement_result

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        #send_http_request(http_commands[f'z_up'])
        time.sleep(6)


        self.get_logger().info(f"Approaching to head {head}")
        location_result = await self.cartesian_location(*locations[f"head{head}"], time=trajectory_time)
        if  location_result != True:
             self.get_logger().error(f"grab from head {head} to exit failed when approaching to head {head}")
             return location_result
        
        # send_http_request(http_commands['z_down'])
      
        # time.sleep(7)

        #send_http_request(http_commands[f'lifter_{head}_up'])
          
        time.sleep(10)

        self.get_logger().info(f"Openning gripper!!!")
        gripperResponse = self.gripper_open()
        if gripperResponse != True:
            self.get_logger().error("something failed when opening the gripper")
            return gripperResponse
        time.sleep(2)

        self.get_logger().info(f"Approaching to head_{head}_back")
        location_result = await self.cartesian_location(*locations[f"head{head}_back"], time=trajectory_time)
        if  location_result != True:
            self.get_logger().error(f"grab from head {head} to exit failed when approaching to head_{head}_back")
            return location_result
        
        return True


    def dispense_noodle(self,request, response):
        pusherid = request.serving
        if (pusherid < 1) or (pusherid > 4):
            response.success = False
            response.message = "Please input id numbers between 1 and 4"
            return response
        
        cscamResponse = self.lifter_master_position(lifterid=pusherid, position=lifter_location["lift_up"])
        if not cscamResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while moving lifter up"
            return response
        
        pusherResponse = self.pusher_master(idPusher=pusherid, command='open',timeToComplete=2000)
        if not pusherResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while openning pusher {pusherid}"
            return response
        
        pusherResponse = self.pusher_master(idPusher=pusherid, command='close',timeToComplete=2600)
        if not pusherResponse:
            response.success = False
            response.message = f"Error dispense noodle {pusherid}. Something failed while closing pusher {pusherid}"
            return response
        
        response.success = True
        response.message = f"Dispense noodle {pusherid} success"
        return response
    
    def cook_noodle(self, request, response):
        cookid = request.serving
        if (cookid < 1) or (cookid > 4):
            response.success = False
            response.message = "Please input id numbers between 1 and 4"
            return response

        #TODO: Function to Boil Water
      
        start_time = time.time()  # Get the current time
        while ((time.time() - start_time) < 30):  # Run for 30 seconds
            cscamResponse = self.lifter_master_position(lifterid=cookid, position=0.0)
            if not cscamResponse:
                response.success = False
                response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking down"
                return response
            time.sleep(0.5)
            cscamResponse = self.lifter_master_position(lifterid=cookid, position=-10.0)
            if not cscamResponse:
                response.success = False
                response.message = f"Error cook noodle {cookid}. Something failed while moving lifter shaking up"
                return response
            time.sleep(0.5)
           
        cscamResponse = self.lifter_master_position(lifterid=cookid, position=lifter_location["lift_up"])
        if not cscamResponse:
            response.success = False
            response.message = f"Error cook noodle {cookid}. Something failed while moving lifter up"
            return response      
        
    
        #TODO: Function to turn off boiler

        response.success = True
        response.message = f"Cook noodle {cookid} success"
        return response

       



def main():
    rclpy.init()
    cooker_robot_sequence = CookerRobotSequence()
    executor = MultiThreadedExecutor()
    rclpy.spin(cooker_robot_sequence, executor=executor)
    cooker_robot_sequence.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
        
