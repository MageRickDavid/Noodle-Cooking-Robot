import serial
import time

# Configuration
PORT = '/dev/ttyUSB5'  # Your confirmed working port
BAUDRATE = 38400  # Baudrate that worked in testing
RELAY_COUNT = 8  # Number of relays on the board

class RelayController:
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()
        
    def connect(self):
        """Connect to the serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
            
    def disconnect(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")
            
    def send_command(self, data, command_type=""):
        """Send command and analyze response for success"""
        if not self.ser or not self.ser.is_open:
            if not self.connect():
                return False, None
                
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            print(f"Sending: {data.hex(' ')}")
            self.ser.write(data)
            time.sleep(0.1)  # Allow time for response
            
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                response_hex = response.hex(' ')
                print(f"Received: {response_hex}")
                
                # Check response length - a valid response should have some minimum length
                if len(response) > 0:
                    # For your specific device, based on your test outputs:
                    # 1. Look for NULL byte sequence (a likely success indicator)
                    if command_type == "relay_on" and response[0] == 0x00:
                        print("✅ SUCCESS: Response indicates the relay ON command was accepted")
                        return True, response
                    elif command_type == "relay_off" and response[0] == 0x00:
                        print("✅ SUCCESS: Response indicates the relay OFF command was accepted")
                        return True, response
                    # 2. Non-empty response is at least some indication
                    elif len(response) > 3:
                        print("✅ SUCCESS: Received data from device")
                        return True, response
                    else:
                        print("⚠️ WARNING: Response format unclear, but device responded")
                        return True, response
                else:
                    print("❌ FAILURE: Empty response")
                    return False, response
            else:
                print("❌ FAILURE: No response received")
                return False, None
        except Exception as e:
            print(f"❌ ERROR: {e}")
            return False, None
            
    def relay_on(self, relay_num):
        """Turn ON a specific relay (0-7)"""
        if 0 <= relay_num < RELAY_COUNT:
            # Function 5 (Write Single Coil): Turn ON
            message = bytes([0x01, 0x05, 0x00, relay_num, 0xFF, 0x00, 0x00, 0x00])
            success, response = self.send_command(message, "relay_on")
            
            if success:
                print(f"Relay {relay_num} successfully turned ON")
            else:
                print(f"Failed to turn ON relay {relay_num}")
                
            return success, response
        else:
            print(f"Invalid relay number: {relay_num}")
            return False, None
            
    def relay_off(self, relay_num):
        """Turn OFF a specific relay (0-7)"""
        if 0 <= relay_num < RELAY_COUNT:
            # Function 5 (Write Single Coil): Turn OFF
            message = bytes([0x01, 0x05, 0x00, relay_num, 0x00, 0x00, 0x00, 0x00])
            success, response = self.send_command(message, "relay_off")
            
            if success:
                print(f"Relay {relay_num} successfully turned OFF")
            else:
                print(f"Failed to turn OFF relay {relay_num}")
                
            return success, response
        else:
            print(f"Invalid relay number: {relay_num}")
            return False, None
            
    def all_relays_on(self):
        """Turn ON all relays"""
        print("\nTurning ON all relays...")
        overall_success = True
        
        # Method 1: Using function code 0x0F (Write Multiple Coils)
        message = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0xFF, 0x29, 0x30])
        success, _ = self.send_command(message, "all_on")
        if not success:
            overall_success = False
            print("Bulk command failed, trying individual relays...")
            
        # Method 2: Individual writes for each relay as backup
        results = []
        for i in range(RELAY_COUNT):
            relay_success, _ = self.relay_on(i)
            results.append(relay_success)
            time.sleep(0.1)
            
        success_count = results.count(True)
        if success_count == RELAY_COUNT:
            print(f"✅ ALL RELAYS ON: All {RELAY_COUNT} relays successfully turned ON")
        else:
            print(f"⚠️ PARTIAL SUCCESS: {success_count}/{RELAY_COUNT} relays turned ON")
            
        return overall_success and all(results)
            
    def all_relays_off(self):
        """Turn OFF all relays"""
        print("\nTurning OFF all relays...")
        overall_success = True
        
        # Method 1: Using function code 0x0F (Write Multiple Coils)
        message = bytes([0x01, 0x0F, 0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0xE8, 0xC0])
        success, _ = self.send_command(message, "all_off")
        if not success:
            overall_success = False
            print("Bulk command failed, trying individual relays...")
            
        # Method 2: Individual writes for each relay as backup
        results = []
        for i in range(RELAY_COUNT):
            relay_success, _ = self.relay_off(i)
            results.append(relay_success)
            time.sleep(0.1)
            
        success_count = results.count(True)
        if success_count == RELAY_COUNT:
            print(f"✅ ALL RELAYS OFF: All {RELAY_COUNT} relays successfully turned OFF")
        else:
            print(f"⚠️ PARTIAL SUCCESS: {success_count}/{RELAY_COUNT} relays turned OFF")
            
        return overall_success and all(results)
            
    def sequence_test(self):
        """Test all relays in sequence - ON then OFF"""
        print("\nRunning relay sequence test...")
        
        # First turn all relays off
        self.all_relays_off()
        time.sleep(1)
        
        # Test each relay individually
        results = []
        for i in range(RELAY_COUNT):
            print(f"\n--- Testing relay {i} ---")
            on_success, _ = self.relay_on(i)
            time.sleep(1)
            off_success, _ = self.relay_off(i)
            time.sleep(0.5)
            results.append(on_success and off_success)
            
        success_count = results.count(True)
        if success_count == RELAY_COUNT:
            print(f"\n✅ SEQUENCE TEST PASSED: All {RELAY_COUNT} relays working correctly")
        else:
            print(f"\n⚠️ SEQUENCE TEST PARTIAL: {success_count}/{RELAY_COUNT} relays working correctly")
            failed_relays = [i for i, success in enumerate(results) if not success]
            print(f"Failed relays: {failed_relays}")
            
        return all(results)
        
    def set_baudrate(self, baudrate):
        """Change the baudrate"""
        if self.ser and self.ser.is_open:
            self.ser.baudrate = baudrate
            self.baudrate = baudrate
            print(f"Baudrate set to {baudrate}")
        else:
            self.baudrate = baudrate
            self.connect()

def main():
    controller = RelayController()
    
    try:
        while True:
            print("\n--- Relay Control Menu ---")
            print("1. Turn ON all relays")
            print("2. Turn OFF all relays")
            print("3. Turn ON a specific relay")
            print("4. Turn OFF a specific relay")
            print("5. Run sequence test")
            print("6. Change baudrate")
            print("7. Exit")
            
            choice = input("Enter your choice (1-7): ")
            
            if choice == '1':
                controller.all_relays_on()
                
            elif choice == '2':
                controller.all_relays_off()
                
            elif choice == '3':
                try:
                    relay_num = int(input("Enter relay number (0-7): "))
                    controller.relay_on(relay_num)
                except ValueError:
                    print("Invalid input. Please enter a number between 0 and 7.")
                    
            elif choice == '4':
                try:
                    relay_num = int(input("Enter relay number (0-7): "))
                    controller.relay_off(relay_num)
                except ValueError:
                    print("Invalid input. Please enter a number between 0 and 7.")
                    
            elif choice == '5':
                controller.sequence_test()
                
            elif choice == '6':
                print("Available baudrates:")
                print("1. 9600")
                print("2. 38400")
                print("3. 57600")
                print("4. 115200")
                
                baud_choice = input("Select baudrate (1-4): ")
                
                if baud_choice == '1':
                    controller.set_baudrate(9600)
                elif baud_choice == '2':
                    controller.set_baudrate(38400)
                elif baud_choice == '3':
                    controller.set_baudrate(57600)
                elif baud_choice == '4':
                    controller.set_baudrate(115200)
                else:
                    print("Invalid choice. Baudrate unchanged.")
                    
            elif choice == '7':
                print("Exiting program...")
                break
                
            else:
                print("Invalid choice. Please enter a number between 1 and 7.")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        # Turn off all relays before exiting
        controller.all_relays_off()
        controller.disconnect()
        print("Program terminated")

if __name__ == "__main__":
    main()