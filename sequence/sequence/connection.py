import socket
import subprocess
import sys
import os
from threading import Thread

class SocketConnection:
    """
    A simple socket connection class that handles running Python files.
    """
    def __init__(self, host: str = 'localhost', port: int = 12345):
        self.host = host
        self.port = port
        self.socket = None
        self.test_file = 'test.py'  # Default test file

    def start_server(self) -> None:
        """Start the server and listen for client connections."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.socket.bind(('', self.port))
            self.socket.listen(1)
            print(f"Server listening on port {self.port}")
            
            while True:
                client_socket, address = self.socket.accept()
                print(f"Connected to client: {address}")
                self._handle_client(client_socket)
                
        except KeyboardInterrupt:
            print("\nServer shutdown by user")
        finally:
            self.close()

    def start_client(self, com: str) -> None:
        """Start the client and connect to server."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            self.socket.connect((self.host, self.port))
            print(f"Connected to server at {self.host}:{self.port}")
            
            while True:
                try:
                    # command = input("\nEnter command ('run' to execute test.py, 'quit' to exit): ").strip()
                    command = input(com).strip()
                    
                    if not command:
                        continue
                        
                    if command.lower() == 'quit':
                        break
                        
                    self.socket.send(command.encode('utf-8'))
                    self._receive_output()
                    
                except KeyboardInterrupt:
                    print("\nInterrupted by user. Enter 'quit' to exit properly.")
                    
        except ConnectionRefusedError:
            print("Could not connect to server. Make sure the server is running.")
        finally:
            self.close()

    def _handle_client(self, client_socket: socket.socket) -> None:
        """Handle client commands and execute test.py when requested."""
        try:
            while True:
                command = client_socket.recv(1024).decode('utf-8')
                
                if not command or command.lower() == 'quit':
                    break
                    
                if command.lower() == 'run':
                    if not os.path.exists(self.test_file):
                        response = f"Error: {self.test_file} not found.\n"
                        client_socket.send(response.encode('utf-8'))
                        continue
                        
                    print(f"Executing {self.test_file}")
                    self._execute_file(client_socket)
                else:
                    response = "Invalid command. Use 'run' to execute test.py or 'quit' to exit.\n"
                    client_socket.send(response.encode('utf-8'))
                    
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            client_socket.close()

    def _execute_file(self, client_socket: socket.socket) -> None:
        """Execute test.py and stream its output to the client."""
        process = subprocess.Popen(
            [sys.executable, self.test_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Create and start output streaming threads
        output_thread = Thread(target=self._stream_output, args=(process.stdout, client_socket, "OUTPUT"))
        error_thread = Thread(target=self._stream_output, args=(process.stderr, client_socket, "ERROR"))
        
        output_thread.start()
        error_thread.start()
        
        # Wait for process to complete
        process.wait()
        output_thread.join()
        error_thread.join()
        
        # Send completion message
        client_socket.send("\nExecution complete.\n".encode('utf-8'))

    def _stream_output(self, pipe, client_socket: socket.socket, stream_type: str) -> None:
        """Stream process output to the client."""
        for line in pipe:
            if line:
                message = f"ERROR: {line}" if stream_type == "ERROR" else line
                try:
                    client_socket.send(message.encode('utf-8'))
                except:
                    break

    def _receive_output(self) -> None:
        """Receive and display output from the server."""
        while True:
            try:
                response = self.socket.recv(1024).decode('utf-8')
                if not response:
                    break
                    
                print(response, end='', flush=True)
                
                if "Execution complete" in response:
                    break
                    
            except socket.error as e:
                print(f"\nError receiving data: {e}")
                break

    def close(self) -> None:
        """Close the socket connection."""
        if self.socket:
            self.socket.close()
            print("\nConnection closed.")
            self.socket = None

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: python script.py [server|client]")
        sys.exit(1)
        
    mode = sys.argv[1].lower()
    connection = SocketConnection()
    
    if mode == 'server':
        connection.start_server()
    elif mode == 'client':
        connection.start_client()
    else:
        print("Invalid mode. Use 'server' or 'client'")