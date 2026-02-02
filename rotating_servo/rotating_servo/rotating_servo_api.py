import serial
import time
import numpy as np


def compute_position_speed_packet(position, speed):
    position_integer = abs(position)*100 #degree
    speed_integer = abs(speed)*10 #rpm
    if position < 0:
        direction = 0x01
    else:
        direction = 0x00

    position_hex = hex(position_integer)
    speed_hex = hex(speed_integer)

    position_first = position_hex[:4]
    position_second = '0x'+position_hex[4:]

    speed_first = speed_hex[:4]
    speed_second = '0x'+speed_hex[4:]

    small_packet = [hex(direction), position_first, position_second, speed_first, speed_second]
    print(f"The small packet is {small_packet}")

    return small_packet





    

    



def compute_position_time_packet(position,time):
    position_integer = int(abs(position)*100) #position 0 - 65533 degree
    time_integer = int(abs(time)*10) #time 1 - 255 seconds

    position_hex = f'0x{position_integer:04x}'
    time_hex = hex(time_integer)
    
    position_first = position_hex[:4]
    position_second = '0x'+position_hex[4:]
    
    if position < 0:
        direction = '0x01'
    else:
        direction = '0x00'
    packet = [direction, position_first, position_second, time_hex]
    print(f"packet sent {packet}")
    return packet

def generate_timebased_position_control_packet(time_based_position_packet):
    time_based_position_packet = [0x00, 0x06,0x02] + [int(number,16) for number in time_based_position_packet]
    checksum = calculate_checksum2(time_based_position_packet)
    complete_packet = [0xFF, 0xFE] + time_based_position_packet[0:2] + [int(checksum,16)] + time_based_position_packet[2:]
    return complete_packet


def command_timebased_position_packet(serial_port, position, time):
    partial_packet = compute_position_time_packet(position,time)
    complete_packet = generate_timebased_position_control_packet(partial_packet)
    print(f"the complete packet I am about to send is: {complete_packet}")
    print(f"The packet in hexagesimal: {[hex(byte) for byte in complete_packet]}")
    serial_port.write(bytearray(complete_packet))
   



def calculate_checksum2(packet):
    sum_in_binary = bin(~np.uint8(sum(packet)))
    sum_in_hex = hex(int(sum_in_binary,2))
    checksum = sum_in_hex
    return checksum

def process_feedback_absolute_position(packet):
    direction = int(packet[-3],16) # 0x00 (CCW) or 0x01 (CW) 
    first_part = packet[-2]
    first_part = int(first_part,16)
    first_part = f'0x{first_part:02x}'

    second_part = packet[-1]
    second_part =  int(second_part,16)
    second_part = f'0x{second_part:02x}'

    total_number = '0x' + first_part[-2:] + second_part[-2:]
    integer_number = int(total_number, 16)
    real_position = integer_number / 100 # +- 180 degreess
    if direction > 0: 
        real_position = -real_position
    return real_position

def request_abs_position(serial_port):
    absolute_position_packet = [0x00, 0x02, 0xA9]
    checksum = calculate_checksum2(absolute_position_packet)
    complete_packet = [0xFF,0xFE, absolute_position_packet[0], absolute_position_packet[1],int(checksum,16), absolute_position_packet[2]]
    serial_port.write(bytearray(complete_packet))
    time.sleep(0.1)
    #print("Receiving response...")
    response = receive_response2(serial_port)
    absolute_position = process_feedback_absolute_position(response)
    #print(f"The absolute position is: {absolute_position}")
    return absolute_position



def calculate_checksum(packet):
    checksum = ~sum(packet[2:]) & 0xFF
    return checksum

def send_packet(serial_port, packet):
    # checksum = calculate_checksum(packet)
    # packet.append(checksum)
    serial_port.write(bytearray(packet))
    print(f"Sent: {packet}")

def receive_response(serial_port, buffer_size=64):
    response = serial_port.read(buffer_size)
    
    print(f"Received: {list(response)}")
    return response

def receive_response2(serial_port, buffer_size=64):
        if serial_port.in_waiting > 0:
            response = serial_port.read(serial_port.in_waiting)
            hex_array = [hex(byte) for byte in response]
            #print(f"Received response: {hex_array}")
            return hex_array


    

def reset_device(serial_port):
    # Reset command (example, update as needed)
    reset_packet = [0xFF, 0xFE, 0x00, 0x02, 0xF1, 0x0C]
    send_packet(serial_port, reset_packet)
    time.sleep(0.5)

def CCW_90(serial_port):
    # Reset command (example, update as needed)
    #[header, -ID, -data, -checksum, mode, direction, angle, speed ]
    reset_packet = [0xFF, 0xFE, 0x00, 0x07, 0x47, 0x01, 0x01, 0x23, 0x28, 0x00, 0x64]
    send_packet(serial_port, reset_packet)
    time.sleep(3)

def CW_90(serial_port):
    # Reset command (example, update as needed)
    reset_packet = [0xFF, 0xFE, 0x00, 0x07, 0x48, 0x01, 0x00, 0x23, 0x28, 0x00, 0x64]
    send_packet(serial_port, reset_packet)
    time.sleep(3)
def CCW_180(serial_port):
    # Reset command (example, update as needed)
    reset_packet = [0xFF, 0xFE, 0x00, 0x07, 0xFD, 0x01, 0x00, 0x46, 0x50, 0x00, 0x64]
    send_packet(serial_port, reset_packet)
    time.sleep(5)

def CW_180(serial_port):
    # Reset command (example, update as needed)
    reset_packet = [0xFF, 0xFE, 0x00, 0x07, 0xFC, 0x01, 0x01, 0x46, 0x50, 0x00, 0x64]
    send_packet(serial_port, reset_packet)
    time.sleep(5)

def CW_360(serial_port):
    # Reset command (example, update as needed)
    reset_packet = [ 0xFF, 0xFE, 0x00, 0x06, 0x98, 0x02, 0x01, 0x8C, 0xA0, 0x32]
    send_packet(serial_port, reset_packet)
    time.sleep(0.5)

def main():
    ser = serial.Serial(
        port='/dev/TorqueUSBZ',          # Update with your COM port
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    try:
        # Ensure the serial port is open
        if not ser.is_open:
            ser.open()

        #compute_position_speed_packet(180,5)
        #command_timebased_position_packet(serial_port=ser,position=-1.06,time=5)
        #time.sleep(5.5)
        #reset_device(ser)
        ##time.sleep(0.2)
        ##command_timebased_position_packet(-360, 5)
        #
        #
        ##compute_position_time_packet(-360, 5)
        request_abs_position(ser)

        # send_packet(ser, [0xFF, 0xFE, 0x00, 0x02, 0x54, 0xA9])
        # time.sleep(0.5)
        # print("Receiving response...")
        # receive_response2(ser)
        

        # # print("Resetting device...")
        # # reset_device(ser)
        

        # # Example 1: Set ID0 to 115,200 bps
        # print("Setting ID0 to 115,200 bps...")
        # packet = [0xFF, 0xFE, 0x00, 0x03, 0xE8, 0x07, 0x0D]
        # send_packet(ser, packet)
        # time.sleep(0.1)

        # # Example 2: Send Ping to ID0
        # print("Sending Ping to ID0...")
        # packet = [0xFF, 0xFE, 0x00, 0x02, 0x2D, 0xD0]
        # send_packet(ser, packet)
        # time.sleep(0.1)


        # CCW_180(ser)
        # time.sleep(1)
        # reset_device(ser)

        # # CW_90(ser)
        # # time.sleep(1)
        # # reset_device(ser)

        # # CCW_90(ser)
        # # time.sleep(1)
        # # reset_device(ser)
        # # ##CW_360(ser)
       
        # print("Receiving response...")
        # response = receive_response(ser)

    except Exception as e:
        print(f"Error: {e}")
    
    
    # finally:
    #     if ser.is_open:
    #         ser.close()
    #     print("Serial port closed.")

if __name__ == "__main__":
    main()
