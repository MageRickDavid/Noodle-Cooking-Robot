# from connection import SocketConnection
import socket
import time

class Client:
    def __init__(self, host='172.30.1.52', port=12345):
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        """Connect to the server."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.host, self.port))
            print(f"Connected to server at {self.host}:{self.port}")
            return True
        except ConnectionRefusedError:
            print("Could not connect to server. Make sure the server is running.")
            return False

    def send_command(self, command: str):
        """Send a command to the server and receive response."""
        try:
            self.socket.send(command.encode('utf-8'))
            while True:
                response = self.socket.recv(1024).decode('utf-8')
                if not response:
                    break
                print(response, end='', flush=True)
                if "Execution complete" in response:
                    break
        except Exception as e:
            print(f"Error sending command: {e}")

    def close(self):
        """Close the connection."""
        if self.socket:
            self.socket.close()
            print("\nConnection closed.")

