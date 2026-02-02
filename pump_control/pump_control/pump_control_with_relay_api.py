
import serial
import time

# Define serial port and baud rate
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with the correct serial port
BAUD_RATE = 9600

# Commands (hex format)
COMMANDS = {
    "read_status": b'\x55\x56\x00\x00\x00\x00\x00\x00\xAB',
    "relay_open": {
        1: b'\x55\x56\x00\x00\x00\x01\x01\xAD',
        2: b'\x55\x56\x00\x00\x00\x02\x01\xAE',
        3: b'\x55\x56\x00\x00\x00\x03\x01\xAF',
        4: b'\x55\x56\x00\x00\x00\x04\x01\xB0',
    },
    "relay_close": {
        1: b'\x55\x56\x00\x00\x00\x01\x02\xAE',
        2: b'\x55\x56\x00\x00\x00\x02\x02\xAF',
        3: b'\x55\x56\x00\x00\x00\x03\x02\xB0',
        4: b'\x55\x56\x00\x00\x00\x04\x02\xB1',
    },
}

def send_command(serial_connection, command):
    """Send a command to the relay module."""
    serial_connection.write(command)
    time.sleep(0.1)  # Wait for response
    response = serial_connection.read_all()
    return response

def main():
    # Initialize serial connection
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print("Connected to relay module.")
        while True:
            print("\nOptions:")
            print("1 - Open a Relay")
            print("2 - Close a Relay")
            print("3 - Read Relay Status")
            print("4 - Exit")

            choice = input("Enter your choice: ").strip()

            if choice == "1":
                try:
                    relay_num = int(input("Enter the relay number (1-4): ").strip())
                    if relay_num not in COMMANDS["relay_open"]:
                        print("Invalid relay number. Please enter a number between 1 and 4.")
                        continue
                    response = send_command(ser, COMMANDS["relay_open"][relay_num])
                    print(f"Relay {relay_num} opened: {response.hex()}")
                except ValueError:
                    print("Invalid input. Please enter a number between 1 and 4.")

            elif choice == "2":
                try:
                    relay_num = int(input("Enter the relay number (1-4): ").strip())
                    if relay_num not in COMMANDS["relay_close"]:
                        print("Invalid relay number. Please enter a number between 1 and 4.")
                        continue
                    response = send_command(ser, COMMANDS["relay_close"][relay_num])
                    print(f"Relay {relay_num} closed: {response.hex()}")
                except ValueError:
                    print("Invalid input. Please enter a number between 1 and 4.")

            elif choice == "3":
                response = send_command(ser, COMMANDS["read_status"])
                print(f"Relay status: {response.hex()}")

            elif choice == "4":
                print("Exiting program.")
                break

            else:
                print("Invalid choice. Please select a valid option.")

if __name__ == "__main__":
    main()