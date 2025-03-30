import serial
import time
import threading

class ESP32Reader:
    def __init__(self, port, baudrate=115200, name=None, latest_values=None, index=None):
        """
        Initialize the ESP32Reader with a serial port and optional name
        
        :param port: Serial port (e.g., '/dev/ttyUSB0')
        :param baudrate: Baud rate for communication (default: 115200)
        :param name: Optional name for the board (defaults to port name)
        :param latest_values: Shared list to store the latest value for each board
        :param index: Index in latest_values where this board's data is stored
        """
        self.port = port
        self.baudrate = baudrate
        self.name = name if name else port
        self.running = False
        self.latest_values = latest_values  # Shared list to store the most recent values
        self.index = index  # Index for this board in the shared list
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            # Ensure the port is open
            if not self.ser.is_open:
                self.ser.open()
        except serial.SerialException as e:
            print(f"Error opening port {self.port}: {e}")
            self.ser = None
        self.thread = None

    def start_reading(self):
        """
        Start the reading thread if the serial port is available
        """
        if self.ser is None:
            print(f"Cannot start reading on {self.name}: No connection")
            return
        if not self.ser.is_open:
            print(f"Serial port {self.name} is not open")
            return
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        print(f"Started reading from {self.name}")

    def read_loop(self):
        """
        Continuously read data from the serial port and update the latest value
        """
        while self.running and self.ser:
            try:
                if self.ser.in_waiting > 0:  # Check if there are bytes to read
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        try:
                            value = int(data)  # Convert received data to integer
                            if self.latest_values is not None and self.index is not None:
                                # Ensure the index is within bounds
                                if 0 <= self.index < len(self.latest_values):
                                    self.latest_values[self.index] = value  # Update the latest value
                                    print(f"{self.name} received: {value}")
                                else:
                                    print(f"Invalid index {self.index} for latest_values")
                            else:
                                print(f"{self.name} received: {value} (no storage assigned)")
                        except ValueError:
                            print(f"{self.name}: Received non-integer data: {data}")
            except serial.SerialException as e:
                print(f"Error reading from {self.name}: {e}")
                self.running = False
            except Exception as e:
                print(f"Unexpected error in {self.name}: {e}")
                self.running = False
            time.sleep(0.001)  # Small delay to avoid excessive CPU usage

    def get_latest_value(self):
        """
        Return the latest value for this board from the shared list
        
        :return: Latest value or None if not available
        """
        if self.latest_values and 0 <= self.index < len(self.latest_values):
            return self.latest_values[self.index]
        return None

    def stop_reading(self):
        """
        Stop the reading thread and close the serial connection
        """
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)  # Wait for the thread to finish with a timeout
            self.thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Stopped reading from {self.name} and closed serial port")
        else:
            print(f"Stopped reading from {self.name} (serial port already closed or not opened)")
