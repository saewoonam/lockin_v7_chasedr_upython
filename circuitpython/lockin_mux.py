# Write your code here :-)
# Write your code here :-)
import supervisor
import microcontroller
import busio
import board
from digitalio import DigitalInOut, Direction, Pull
import json

uart0 = busio.UART(board.A1, board.A0, baudrate=9600)
pinA0 = DigitalInOut(board.D2)
pinB0 = DigitalInOut(board.D3)
pinA0.direction = Direction.OUTPUT
pinB0.direction = Direction.OUTPUT

pinA0.value = False
pinB0.value = False

uart1 = busio.UART(board.TX, board.RX)
pinA1 = DigitalInOut(board.SDA)
pinB1 = DigitalInOut(board.SCL)
pinA1.direction = Direction.OUTPUT
pinB1.direction = Direction.OUTPUT



def set_mux(pinA, pinB, value):
    pinA.value = 1 if value & 0x1 else 0
    pinB.value = 1 if value & 0x2 else 0

unique_id = microcontroller.cpu.uid
name = 'lockin_mux'

def writeUSB(item):
    print(json.dumps(item))

def main():

    while True:
        if supervisor.runtime.serial_bytes_available:
            value = input().strip()
            params = value.split()
            if len(params):
                value = params[0].upper();
            if len(params)==1:
                params = []
            elif len(params)>1:
                params = params[1:];
                print('params', params)
            #print(f"Received: {value.split()}\r")
            if value.upper()=='*IDN?':
                writeUSB(unique_id)
            elif value.upper()=='*NAME?':
                writeUSB(name)
            elif value.upper() == 'Q':
                break;
            #elif value.upper()=='R?' or value.upper()=='RA?' or value.upper()=='LAST?' or value.upper()=='BIAS?' or :
            elif value.upper() in ['R?', 'RA?', 'LAST?', 'BIAS?', 'BIAS']:
                if (len(params)>0):
                    #print(params);
                    channel = int(params[0])
                    if value.upper()=='BIAS':
                        cmd = 'BIAS '+params[1]
                    else:
                        cmd = value.upper();
                    if (channel>=0) and (channel<8):
                        #print("got R, channel:", channel)
                        uart = uart1 if channel & 4 else uart0
                        pinA = pinA1 if channel & 4 else pinA0
                        pinB = pinB1 if channel & 4 else pinB0
                        set_mux(pinA, pinB, channel & 3);  # Use lowest two bits
                        cmd = cmd + '\n'
                        cmd = cmd.encode()
                        uart.write(cmd)
                        failed_counter = 0
                        while True:
                            response = uart.readline()
                            if response is None:  # sometimes the request to readline happens to fast... try again
                                if failed_counter == 5:
                                    response = 'failed to read'
                                    break;
                                failed_counter += 1
                            else:
                                break
                        writeUSB(response);
                    else:
                        writeUSB('bad command')
                else:
                    writeUSB('bad command')
            else:
                writeUSB('bad command')

main()
