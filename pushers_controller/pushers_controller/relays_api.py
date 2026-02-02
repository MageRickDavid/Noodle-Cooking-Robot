from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from pymodbus.pdu import ExceptionResponse
import serial


# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your actual port (Linux: /dev/ttyUSB0)
BAUD_RATE = 9600
PARITY = 'N'
STOP_BITS = 1
BYTE_SIZE = 8
TIMEOUT = 1  # Timeout in seconds
DEVICE_ADDRESS = 1  # Change according to your device's MODBUS address

# Initialize Modbus RTU client
client = ModbusSerialClient(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    parity=PARITY,
    stopbits=STOP_BITS,
    bytesize=BYTE_SIZE,
    timeout=TIMEOUT
)

# Connect to the Modbus device
try:
    if not client.connect():
        raise ConnectionError("Failed to connect to the Modbus device.")
except serial.SerialException as e:
    print(f"Serial Error: {e}")
    exit(1)
except ConnectionError as e:
    print(e)
    exit(1)

def read_relay_status(device_address, relay_address):
    """Read the status of a specific relay."""
    try:
        response = client.read_coils(address=relay_address, count=1, slave=device_address)
        if response.isError():
            print(f"Error reading relay {relay_address} on device {device_address}: {response}")
            return None
        return response.bits[0]  # Returns True (ON) or False (OFF)
    except ModbusException as e:
        print(f"Modbus error: {e}")
        return None

def write_relay(device_address, relay_address, value):
    """Turn a relay ON or OFF."""
    try:
        response = client.write_coil(address=relay_address, value=value, slave=device_address)
        if response.isError():
            print(f"Error writing to relay {relay_address} on device {device_address}: {response}")
            return False
        return True
    except ModbusException as e:
        print(f"Modbus error: {e}")
        return False

def write_multiple_relays(device_address, relay_actions):
    """Turn multiple relays ON or OFF at the same time."""
    try:
        for relay_address, value in relay_actions.items():
            if not write_relay(device_address, relay_address, value):
                return False
        return True
    except ModbusException as e:
        print(f"Modbus error: {e}")
        return False

def read_all_relays(device_address, num_relays=8):
    """Read the status of all relays."""
    try:
        response = client.read_coils(address=0, count=num_relays, slave=device_address)
        if response.isError():
            print(f"Error reading all relays on device {device_address}: {response}")
            return None
        return response.bits  # Returns a list of relay statuses (True/False)
    except ModbusException as e:
        print(f"Modbus error: {e}")
        return None

# Main function to interact with the user
def main():
    while True:
        print("\nOptions:")
        print("1 - Control a single relay")
        print("2 - Control multiple relays")
        print("3 - Read all relay statuses")
        print("4 - Exit")

        choice = input("Enter your choice: ").strip()

        if choice == "1":
            relay_address = input("Enter the relay address (0-7): ").strip()
            try:
                relay_address = int(relay_address)
                if relay_address < 0 or relay_address > 7:
                    print("Invalid relay address. Please enter a number between 0 and 7.")
                    continue
            except ValueError:
                print("Invalid input. Please enter a number between 0 and 7.")
                continue

            action = input("Enter the action (ON/OFF): ").strip().upper()
            if action not in ["ON", "OFF"]:
                print("Invalid action. Please enter 'ON' or 'OFF'.")
                continue

            value = action == "ON"  # Converts 'ON' to True, 'OFF' to False
            if write_relay(DEVICE_ADDRESS, relay_address, value):
                print(f"Relay {relay_address} turned {action}.")

            status = read_relay_status(DEVICE_ADDRESS, relay_address)
            if status is not None:
                print(f"Relay {relay_address} status: {'ON' if status else 'OFF'}")

        elif choice == "2":
            relay_actions = {}
            while True:
                relay_address = input("Enter the relay address (0-7), or 'done' to finish: ").strip()
                if relay_address.lower() == 'done':
                    break

                try:
                    relay_address = int(relay_address)
                    if relay_address < 0 or relay_address > 7:
                        print("Invalid relay address. Please enter a number between 0 and 7.")
                        continue
                except ValueError:
                    print("Invalid input. Please enter a number between 0 and 7.")
                    continue

                action = input(f"Enter the action for relay {relay_address} (ON/OFF): ").strip().upper()
                if action not in ["ON", "OFF"]:
                    print("Invalid action. Please enter 'ON' or 'OFF'.")
                    continue

                value = action == "ON"  # Converts 'ON' to True, 'OFF' to False
                relay_actions[relay_address] = value

            if relay_actions:
                if write_multiple_relays(DEVICE_ADDRESS, relay_actions):
                    print("✅ Multiple relays updated successfully.")

        elif choice == "3":
            all_relays_status = read_all_relays(DEVICE_ADDRESS)
            if all_relays_status is not None:
                print("All relay statuses:")
                for i, status in enumerate(all_relays_status):
                    print(f"Relay {i}: {'ON' if status else 'OFF'}")

        elif choice == "4":
            print("Exiting...")
            break

        else:
            print("Invalid choice. Please enter a valid option.")

    client.close()

if __name__ == "__main__":
    main()