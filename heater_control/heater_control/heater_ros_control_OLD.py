import rclpy
import rclpy.duration
import rclpy.executors
import rclpy.logging
import numpy as np
import time
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gripper_interfaces.srv import HeaterValve
from gripper_interfaces.msg import Temperature
import minimalmodbus
import serial
from .heater_control_api import control_relay

COMMAND = {
    "on": 1,
    "off": 0
}

class HeaterValveNode(Node):
    def __init__(self):
        super().__init__('HeaterValve_node')
        self.tty = '/dev/Heater'
        # Configure Modbus instrument
        #self.connect_serial()
        self.call_back_groups_setup()
        self.service_setup()
        self.timer_setup()
        self.subscriber_setup()
        self.get_logger().info(f"HeaterValve relays are on Successfully new!!!!!")
        self.flag_for_sequence_node_monitoring = False
        self.relay_state = {
            1: 1,
            2: 0,
            3: 1,
            4: 1,
            5: 0,
            6: 1,
            7: 0,
            8: 0,
        }
        

        
    def connect_serial(self):
        try:
            self.relay_board = minimalmodbus.Instrument(self.tty, 1)
            self.relay_board.serial.baudrate = 115200
            self.relay_board.serial.bytesize = 8
            self.relay_board.serial.parity = serial.PARITY_NONE
            self.relay_board.serial.stopbits = 1
            self.relay_board.serial.timeout = 0.5
           
            self.get_logger().info("Minimal modbus successfully connected")
            return True
        except Exception as e:
            self.get_logger().error("The relay for the heater and valve says: " + str(e))
            return False

    def timer_setup(self):
        self.sending_command = self.create_timer(timer_period_sec = 0.01, callback=self.relay_control, callback_group=self.sending_command_group)

    def call_back_groups_setup(self):
        self.valve_controller_group = MutuallyExclusiveCallbackGroup()
        self.heater_controller_group = MutuallyExclusiveCallbackGroup()
        self.temperature_control_group = MutuallyExclusiveCallbackGroup()
        self.sending_command_group = MutuallyExclusiveCallbackGroup()
        self.boiler_controller_group = MutuallyExclusiveCallbackGroup()
    
    def relay_control(self):
        
        self.get_logger().info(f"new dictionary {self.relay_state}")
       
        for relayId, stateRelay in self.relay_state.items():
            control_relay(port=self.tty,slave_id=1,relay_id=relayId, state=True) if stateRelay == 1 else time.sleep(0.01)
            #time.sleep(1)
        #self.get_logger().info("Sent all true commands")
        time.sleep(0.75)

        for relayId, stateRelay in self.relay_state.items():
            control_relay(port=self.tty,slave_id=1,relay_id=relayId, state=False) if not stateRelay== 0 else time.sleep(0.01)
            # time.sleep(1)
        #self.get_logger().info("Sent all false commands")
        time.sleep(0.75)

        
            
    
    def service_setup(self):
        self.valve_controller_srv = self.create_service(HeaterValve, "/valve_control", self.turn_on_off_valve, callback_group=self.valve_controller_group)
        self.heater_controller_srv = self.create_service(HeaterValve, "/heater_control", self.turn_on_off_heater, callback_group=self.heater_controller_group)
        self.boiler_srv = self.create_service(HeaterValve, "/boiler_control", self.boiler_control, callback_group=self.boiler_controller_group)

    def subscriber_setup(self):
        self.temperature_subscription = self.create_subscription(Temperature, "/thermocouple_temperature", self.temperature_callback, 10)
    

    def boiler_control(self, request, response):
        boiler_id = request.relayid
        command = request.command
        commmand_bool = True if command == "on" else False
        if (boiler_id < 1) or (boiler_id > 2):
            response.success = False
            response.message = "There are only two boilers"
            return response
        if (boiler_id == 1):
            self.relay_state[1] = commmand_bool
            time.sleep(0.1)
            self.relay_state[2] = commmand_bool
            time.sleep(0.1)
            self.relay_state[3] = commmand_bool
            time.sleep(0.1)
        else:
            self.relay_state[4] = commmand_bool
            time.sleep(0.1)
            self.relay_state[5] = commmand_bool
            time.sleep(0.1)
            self.relay_state[6] = commmand_bool
            time.sleep(0.1)
        

        response.message = f"Boiler {command} sent successfully"
        response.success = True
        return response

    def turn_on_off_valve(self, request, response):
            valve_id = request.relayid
            command = request.command
            commmand_bool = True if command == "on" else False
            if (valve_id < 7) or (valve_id > 8):
                response.success = False
                response.message = "Valves belong to relay 6 or 7"
                return response

            self.relay_state[valve_id] = commmand_bool
            refresh_list = self.relay_state.copy()
            time.sleep(0.1)
            if refresh_list[valve_id] != commmand_bool:
                response.success = False
                response.message = "The relay state was not updated"
                return response

            response.message = f"Valve {commmand_bool} sent successfully"
            response.success = True
            return response
    def turn_on_off_heater(self, request, response):
            heater_id = request.relayid
            command = request.command
            #commmand_bool = True if command == "on" else False
            command_bool = COMMAND[command]
            if (heater_id < 1) or (heater_id > 6):
                response.success = False
                response.message = "Heaters belong to relay 0 or 5"
                return response
            
            self.relay_state[heater_id] = command_bool
            self.get_logger().info(f"relay {heater_id} is {self.relay_state[heater_id]}")
            self.get_logger().info(f"current dictionary {self.relay_state}")
            refresh_list = self.relay_state.copy()
            time.sleep(0.1)
            if refresh_list[heater_id] != command_bool:
                response.success = False
                response.message = "The relay state was not updated"
                return response
            response.message = f"Heater {heater_id} state {command_bool} sent successfully"
            response.success = True
            return response
   

    def temperature_callback(self, msg):
        if not self.flag_for_sequence_node_monitoring:
            
            temperature1 = msg.temperature[0]
            if temperature1 <= 80:
                self.relay_state[1] = True
                time.sleep(2) 
                self.relay_state[2] = True
                time.sleep(2) 
                self.relay_state[3] = True
                time.sleep(2) 

            else:
                self.relay_state[1] = False
                self.relay_state[2] = False
                self.relay_state[3] = False
               

            temperature2 = msg.temperature[1]
            if temperature1 <= 80:
                self.relay_state[4] = True
                time.sleep(2) 
                self.relay_state[5] = True
                time.sleep(2) 
                self.relay_state[6] = True
                time.sleep(2) 

            else:
                self.relay_state[4] = False
                self.relay_state[5] = False
                self.relay_state[6] = False





def main():
    rclpy.init()
    heaterValveNode = HeaterValveNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(heaterValveNode, executor=executor)
    except KeyboardInterrupt:
    #    heaterValveNode.turn_off_relay(1)
    #    heaterValveNode.turn_off_relay(2)
    #    heaterValveNode.turn_off_relay(3)
    #    heaterValveNode.turn_off_relay(4)
    #    heaterValveNode.turn_off_relay(5)
    #    heaterValveNode.turn_off_relay(6)
    #    heaterValveNode.turn_off_relay(7)
    #    heaterValveNode.turn_off_relay(8)
       print('RELAYS SHUTDOWN SUCCESSFULLY!!!')
       heaterValveNode.get_logger().error("ErrorHeaterValve")


if __name__ == '__main__':
    main()        



        
        

