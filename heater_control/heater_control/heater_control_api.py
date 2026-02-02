import minimalmodbus
import serial
import time

def control_relay(port, slave_id, relay_id, state):
    """
    Control a specific relay.
    
    Args:
        port (str): Serial port (e.g., '/dev/ttyUSB1')
        slave_id (int): Modbus slave ID of the relay board
        relay_id (int): ID of the relay to control (1-8)
        state (bool): True for ON, False for OFF
    """
    try:
        relay_board = minimalmodbus.Instrument(port, slave_id)
        relay_board.serial.baudrate = 115200
        relay_board.serial.bytesize = 8
        relay_board.serial.parity = serial.PARITY_NONE
        relay_board.serial.stopbits = 1
        #relay_board.serial.timeout = 0.5
        relay_board.serial.timeout = 0.1
        try:
            relay_board.write_bit(relay_id, 1 if state else 0, functioncode=5)
            #print(f"Relay {relay_id} turned {'ON' if state else 'OFF'}")
        except Exception as e:
            # if "Too short Modbus RTU response" in str(e) and "b'\\x00'" in str(e):
            #     print(f"Relay {relay_id} command sent (ignoring expected response error)")
            # else:
            #     raise e
            pass
                
    except Exception as e:
        print(f"Error: {e}")
        return False
    
    return True

def continuous_control():
    port = '/dev/Heater'
    slave_id = 1
    relay_ids = [2,5]
    relay_id_off = [ 1, 4,6, 3,7]
    
    while True:
        for relay_id in relay_ids:
            control_relay(port, slave_id, relay_id, True)
        time.sleep(1)  # Adjust the delay as needed
        
        for relay_id in relay_id_off:
          control_relay(port, slave_id, relay_id, False)
        time.sleep(1)

if __name__ == "__main__":
    continuous_control()