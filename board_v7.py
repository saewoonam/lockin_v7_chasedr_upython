from machine import Pin, idle, UART, RTC
import time
import rp2
import gc
from sys import stdin, stdout, exit
import select
from HX710C import HX710C
from ulab import numpy as np
rtc = RTC()

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
poll = select.poll()
poll.register(uart, select.POLLIN)

ONCE = False
DEBUG = True
last = []
alpha = 0.2

def readUSBSerial():
    global ONCE, DEBUG, last
    gc.collect()
    while stdin in select.select([stdin], [], [], 0)[0]:
        #print('Got USB serial message')
        gc.collect()
        #print('in select')
        cmd = stdin.readline()
        #print(type(cmd), repr(cmd))
        cmd = cmd.strip().upper()
        doCommand(cmd)

def uartresponder(msg):
    print('uart response', repr(msg));
    msg += '\n'
    uart.write(msg)
    
def readUart():
    global ONCE, DEBUG, last
    gc.collect()
    events = poll.poll(0)
    for event in events:
        #print('Got USB serial message')
        if event[0]==uart:
            gc.collect()
            print('\nin uart after poll')
            cmd = uart.readline()
            print(type(cmd), repr(cmd))
            cmd = cmd.strip().upper()
            print('command: ', cmd);
            if type(cmd)==bytes:
                cmd = cmd.decode()
            doCommand(cmd, uartresponder)

def doCommand(cmd, responder=print):
    global ONCE, DEBUG, last, p9, counter
    #print('do Command', cmd)
    if len(cmd):  # respond to command
        args = cmd.split();
        cmd = args[0]
        if (cmd[0]=='Q'):
            responder('Got Q')
            exit(0)
        elif (cmd=='PING'):
            responder('PONG')
        elif (cmd=="R?"):
            resp = [last[0], last[1], last[2]]
            responder(f'{resp}')
        elif (cmd=="RA?"):
            resp = [last[0], last[1], last[3]]
            responder(f'{resp}')
        elif (cmd=="LAST?"):
            responder(f'{last}')
        elif (cmd=="BIAS?"):
            response = p9.value()
            responder(f'{response}')
        elif (cmd=="BIAS"):
            if len(args)>1:
                print('args', args);
                channel = int(args[1])
                if channel:
                    p9.on()
                else:
                    p9.off()
                counter = 0
                responder("done")
            else:
                responder("bad channel for bias");
        elif (cmd=="DEBUG"):
            DEBUG = not DEBUG
            responder(f"DEBUG: {DEBUG}")
        elif (cmd=="DEBUG?"):
            responder(f'{DEBUG}')
        else:
            responder('bad command');

print('name', __name__)
if (__name__ == 'board_v7') or (__name__=='__main__'):
    p6 = Pin(26, Pin.OUT) # high = bias on
    p7 = Pin(27, Pin.OUT) # polarity 100k 
    p8 = Pin(28, Pin.OUT) # polarity sensor
    p9 = Pin(29, Pin.OUT) # low/high bias control
    
    p6.on()  # turn on bias
    p7.on() 
    p8.on()
    p9.on()  # low bias
    
    adc = HX710C(2, 6)
    conversion = 1e6 * 2.5 / (1<<24) / 128  # converts to uV
    counter = 0
    
    def readN_reset(N, skip=4, reset=False):
        if reset:
            adc.reset_sm()
            adc.sm.active(1)
        for count in range(skip):
            adc.read()
        while True:
            readings = []
            for count in range(N):
                #state = disable_irq()
                raw = adc.read()
                #enable_irq(state)
                reading = [raw[0], #/ (1<<24) * 2.5 / 128 * 1e6,
                           raw[1], #/ (1<<24) * 2.5 / 128 * 1e6,
                           ]
                #print(reading)
                readings.append(reading)
            readings = np.array(readings)
            # check if there may be bad datat points... if max-min is too big...
            #print( np.sum((np.max(readings, axis=0)-np.min(readings, axis=0))>8000) )
            if     np.sum((np.max(readings, axis=0)-np.min(readings, axis=0)) > 8000) == 0:
                break
            #else:
            #    print('bad reading', '*'*40, readings)
        
        return np.array(readings)
    def toggle678():
        global last, alpha, counter
        alpha = 0.2
        adc.sm.active(1)
        toggle_bias = True
        while True:
            readUart();
            if toggle_bias:
                p6.toggle()
            p7.toggle()
            p8.toggle()
            readings = readN_reset(5)
            r1=readings
            readings1 = np.mean(readings, axis=0)
            p7.toggle()
            p8.toggle()
            readings = readN_reset(5)
            r2=readings
            readings2 = np.mean(readings, axis=0)
            
            current_readings = readings1-readings2
            inner_offset = (readings1 + readings2)/2
            #print
            #print(current_readings);
            #print('counter',counter)
            if counter>0:
                diff = current_readings-prev_readings
                diff /= 2.0
                offset = current_readings + prev_readings
                offset /=2
                r = diff[1]/diff[0] * 100
                if counter == 1:
                    ewa = r;
                #if (abs(offset[0])>8000):
                if False:
                    # Don't update ewa if the offset is off.
                    #print(f'{counter} {r:6.4} {ewa:6.4} {offset[0]:>6.3} {offset[1]:>6.5} {inner_offset[0]:>6.3} {inner_offset[1]:>6.5}')
                    print(f'{counter} {time.time()} {r:6.2f} {ewa:6.2f} {offset[0]:>9.2f} {offset[1]:>9.2f} {inner_offset[0]:>9.2f} {inner_offset[1]:>9.2f}')
                    print(r1, r2)
                    print(np.mean(r1, axis=0), np.mean(r2, axis=0))
                    toggle_bias = False
                    current_readings = prev_readings # setup to try again
                else:
                    ewa = r*alpha + (1-alpha)*ewa                
                    #print(f'{counter} {r:6.4} {ewa:6.4} {offset[0]:>6.3} {offset[1]:>6.5} {inner_offset[0]:>6.3} {inner_offset[1]:>6.5}', end='\r')
                    print(f'{counter} {time.time()} {r:6.4f} {ewa:6.4f} {diff} {offset[0]:>9.2f} {offset[1]:>9.2f} {inner_offset[0]:>9.2f} {inner_offset[1]:>9.2f}', end='\r')
                    toggle_bias = True
                last = [counter, time.time(), r, ewa]
            #print(current_readings)
            prev_readings = current_readings
            counter += 1
    toggle678()
