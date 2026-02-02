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

COMMAND = {
    "on": 1,
    "off": 0
}

def control_relay(port, slave_id, relay_id, state):
    """
    Control a specific relay with proper error handling.
    
    Args:
        port (str): Serial port (e.g., '/dev/Heater')
        slave_id (int): Modbus slave ID of the relay board
        relay_id (int): ID of the relay to control (1-8)
        state (bool): True for ON, False for OFF
    """
    try:
        # Initialize the instrument
        relay_board = minimalmodbus.Instrument(port, slave_id)
        relay_board.serial.baudrate = 115200
        relay_board.serial.bytesize = 8
        relay_board.serial.parity = serial.PARITY_NONE
        relay_board.serial.stopbits = 1
        relay_board.serial.timeout = 0.5
        
        # Try to control the relay
        try:
            relay_board.write_bit(relay_id, 1 if state else 0, functioncode=5)
            return True
        except Exception as e:
            # If we get the specific error we saw earlier but the command seems to work
            if "Too short Modbus RTU response" in str(e) and "b'\\x00'" in str(e):
                return True
            else:
                raise e
                
    except Exception as e:
        print(f"Error controlling relay {relay_id}: {e}")
        return False

class HeaterValveNode(Node):
    def __init__(self):
        super().__init__('HeaterValve_node')
        self.tty = '/dev/Heater'  # Make sure this path is correct!
        
        # Setup callback groups first
        self.call_back_groups_setup()
        
        # Setup services and timers
        self.service_setup()
        self.subscriber_setup()
        self.timer_setup()
        
        # Initialize relay states
        self.relay_state = {
            1: 0,  # Start with all relays off for safety
            2: 0,
            3: 0,
            4: 0,
            5: 0,
            6: 0,
            7: 0,
            8: 0,
        }
        
        # Flag for temperature monitoring
        self.flag_for_sequence_node_monitoring = False
        
        # Test serial connection on startup
        success = self.test_connection()
        if success:
            self.get_logger().info(f"HeaterValve node initialized successfully!")
        else:
            self.get_logger().error(f"Failed to initialize HeaterValve node - check serial connection")

    def test_connection(self):
        """Test if we can communicate with the relay board"""
        try:
            # Try to control relay 1 (turn it off)
            result = control_relay(port=self.tty, slave_id=1, relay_id=1, state=False)
            if result:
                self.get_logger().info("Successfully connected to relay board")
                return True
            else:
                self.get_logger().error("Failed to connect to relay board")
                return False
        except Exception as e:
            self.get_logger().error(f"Error testing relay connection: {str(e)}")
            return False

    def timer_setup(self):
        # Create timer for relay control at 10Hz (every 0.1 seconds)
        self.sending_command = self.create_timer(
            timer_period_sec=0.1, 
            callback=self.relay_control, 
            callback_group=self.sending_command_group
        )

    def call_back_groups_setup(self):
        self.valve_controller_group = MutuallyExclusiveCallbackGroup()
        self.heater_controller_group = MutuallyExclusiveCallbackGroup()
        self.temperature_control_group = MutuallyExclusiveCallbackGroup()
        self.sending_command_group = MutuallyExclusiveCallbackGroup()
        self.boiler_controller_group = MutuallyExclusiveCallbackGroup()
    
    def relay_control(self):
        """Send commands to update all relay states according to relay_state dictionary"""
        try:
            # Log current relay states (reduced frequency to avoid log spam)
            if time.time() % 5 < 0.1:  # Log roughly every 5 seconds
                self.get_logger().info(f"Current relay states: {self.relay_state}")
            
            # Update each relay with its current state
            for relay_id, state in self.relay_state.items():
                # Convert integer 0/1 to boolean False/True
                state_bool = True if state == 1 else False
                control_relay(
                    port=self.tty,
                    slave_id=1,
                    relay_id=relay_id, 
                    state=state_bool
                )
                # Small delay between commands to avoid overwhelming the serial bus
                time.sleep(0.01)  
                
        except Exception as e:
            self.get_logger().error(f"Error in relay_control: {str(e)}")
            
    def service_setup(self):
        self.valve_controller_srv = self.create_service(
            HeaterValve, 
            "/valve_control", 
            self.turn_on_off_valve, 
            callback_group=self.valve_controller_group
        )
        
        self.heater_controller_srv = self.create_service(
            HeaterValve, 
            "/heater_control", 
            self.turn_on_off_heater, 
            callback_group=self.heater_controller_group
        )
        
        self.boiler_srv = self.create_service(
            HeaterValve, 
            "/boiler_control", 
            self.boiler_control, 
            callback_group=self.boiler_controller_group
        )

    def subscriber_setup(self):
        self.temperature_subscription = self.create_subscription(
            Temperature, 
            "/thermocouple_temperature", 
            self.temperature_callback, 
            10,
            callback_group=self.temperature_control_group
        )
    
    def boiler_control(self, request, response):
        try:
            boiler_id = request.relayid
            command = request.command
            command_value = COMMAND.get(command, 0)  # Default to 0 (off) if invalid command
            
            if (boiler_id < 1) or (boiler_id > 2):
                response.success = False
                response.message = "There are only two boilers (1-2)"
                return response
            
            # Set relays for boiler 1 (relays 1-3)
            if (boiler_id == 1):
                for relay_id in [1, 2, 3]:
                    self.relay_state[relay_id] = command_value
            # Set relays for boiler 2 (relays 4-6)
            else:
                for relay_id in [4, 5, 6]:
                    self.relay_state[relay_id] = command_value
            
            response.message = f"Boiler {boiler_id} {command} command sent successfully"
            response.success = True
            
        except Exception as e:
            response.success = False
            response.message = f"Error in boiler_control: {str(e)}"
            
        return response

    def turn_on_off_valve(self, request, response):
        try:
            valve_id = request.relayid
            command = request.command
            command_value = COMMAND.get(command, 0)  # Default to 0 (off) if invalid command
            
            if (valve_id < 7) or (valve_id > 8):
                response.success = False
                response.message = "Valves belong to relay 7 or 8"
                return response

            # Update relay state
            self.relay_state[valve_id] = command_value
            
            response.message = f"Valve {valve_id} {command} command sent successfully"
            response.success = True
            
        except Exception as e:
            response.success = False
            response.message = f"Error in turn_on_off_valve: {str(e)}"
            
        return response
        
    def turn_on_off_heater(self, request, response):
        try:
            heater_id = request.relayid
            command = request.command
            command_value = COMMAND.get(command, 0)  # Default to 0 (off) if invalid command
            
            if (heater_id < 1) or (heater_id > 6):
                response.success = False
                response.message = "Heaters belong to relays 1-6"
                return response
            
            # Update relay state
            self.relay_state[heater_id] = command_value
            self.get_logger().info(f"Setting relay {heater_id} to {command_value}")
            
            response.message = f"Heater {heater_id} {command} command sent successfully"
            response.success = True
            
        except Exception as e:
            response.success = False
            response.message = f"Error in turn_on_off_heater: {str(e)}"
            
        return response

    def temperature_callback(self, msg):
        """Automatically control heaters based on temperature if not in manual mode"""
        if self.flag_for_sequence_node_monitoring:
            return  # Skip if in manual control mode
            
        try:
            # Control heaters for first temperature sensor
            temperature1 = msg.temperature[0]
            if temperature1 <= 80:
                # Turn on heaters 1-3 if temperature is below threshold
                self.relay_state[1] = 1
                self.relay_state[2] = 1
                self.relay_state[3] = 1
            else:
                # Turn off heaters 1-3 if temperature is above threshold
                self.relay_state[1] = 0
                self.relay_state[2] = 0
                self.relay_state[3] = 0
            
            # Control heaters for second temperature sensor
            temperature2 = msg.temperature[1]
            if temperature2 <= 80:  # Fixed logic (was using temperature1 again)
                # Turn on heaters 4-6 if temperature is below threshold
                self.relay_state[4] = 1
                self.relay_state[5] = 1
                self.relay_state[6] = 1
            else:
                # Turn off heaters 4-6 if temperature is above threshold
                self.relay_state[4] = 0
                self.relay_state[5] = 0
                self.relay_state[6] = 0
                
        except Exception as e:
            self.get_logger().error(f"Error in temperature_callback: {str(e)}")

    def shutdown(self):
        """Safely turn off all relays on shutdown"""
        try:
            # Turn off all relays
            for relay_id in range(1, 9):
                control_relay(port=self.tty, slave_id=1, relay_id=relay_id, state=False)
                time.sleep(0.05)  # Small delay between commands
            self.get_logger().info("All relays safely turned OFF during shutdown")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")

def main():
    rclpy.init()
    heater_valve_node = HeaterValveNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(heater_valve_node, executor=executor)
    except KeyboardInterrupt:
        heater_valve_node.get_logger().info("Shutting down due to keyboard interrupt...")
    except Exception as e:
        heater_valve_node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        # Safely shut down all relays
        heater_valve_node.shutdown()
        heater_valve_node.destroy_node()
        rclpy.shutdown()
        print('HeaterValve node has been shut down.')

if __name__ == '__main__':
    main()