from pymodbus.client import ModbusSerialClient

# Modbus Configuration
PORT = "/dev/ttyUSB5"  # Change this if needed (e.g., COM3 on Windows)
BAUDRATE = 115200
SLAVE_ID = 1  # Change if your relay board uses a different ID

# Initialize Modbus RTU client
client = ModbusSerialClient(
    method="rtu",
    port=PORT,
    baudrate=BAUDRATE,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

def control_relay(relay_id, state):
    """
    Control a relay by Modbus ID and ON/OFF state.
    
    relay_id: (1-8)  - The relay number
    state: "on" or "off"
    """
    if not client.connect():
        print("❌ Failed to connect to Modbus device!")
        return False

    try:
        coil_address = relay_id - 1  # Modbus coils use 0-based addressing
        command = state.lower() == "on"
        
        result = client.write_coil(coil_address, command, unit=SLAVE_ID)

        if result.isError():
            print(f"❌ Failed to set Relay {relay_id} to {state.upper()}")
        else:
            print(f"✅ Relay {relay_id} turned {state.upper()}")
    
    except Exception as e:
        print(f"⚠️ Error: {e}")
    
    finally:
        client.close()  # Close connection after sending command

# Interactive Loop
while True:
    user_input = input("\nEnter command (<relay_id> <on/off>, or 'q' to quit): ").strip().lower()
    
    if user_input == "q":
        break

    try:
        relay_id, state = user_input.split()
        relay_id = int(relay_id)

        if relay_id < 1 or relay_id > 8:
            print("⚠️ Relay ID must be between 1 and 8.")
            continue
        if state not in ["on", "off"]:
            print("⚠️ State must be 'on' or 'off'.")
            continue

        control_relay(relay_id, state)
    
    except ValueError:
        print("⚠️ Invalid format! Use: <relay_id> <on/off> (e.g., '3 on').")