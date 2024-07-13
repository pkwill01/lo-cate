import serial
import serial.tools.list_ports

def list_com_ports():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port)

list_com_ports()
# Replace 'COM3' with the actual COM port number you found earlier
port_name = '/dev/ttyACM0'
baud_rate = 9600  # Common baud rate, adjust if necessary


ser = serial.Serial(port=port_name, baudrate=baud_rate)

print(f"Connected to {ser.port}")

# Read data from the serial port
while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
