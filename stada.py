import serial
xbee_port = "COM12" #your xbee usb port
ser = serial.Serial(xbee_port, 9600)

# Read line   
while True:
    #WRITTING
    #input from command
    input_command = input()
    # convert string to byte 
    bytes_input_command = bytes(input_command, 'utf-8')
    # Send character 'S' to start the program
    ser.write(bytes_input_command)

    #READING
    read_xbee = ser.readline()#reading com
    print(read_xbee)