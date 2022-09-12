from machine import Pin, idle, UART, RTC
import time
import rp2
import gc
from sys import stdin, stdout, exit
import select

rtc = RTC()

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
poll = select.poll()
poll.register(uart, select.POLLIN)

# from https://github.com/robert-hh/hx711/blob/master/hx711_pio.py
class HX710C:
    def __init__(self, clk, data):
        self.pSCK = clk
        self.pOUT = data
        self.pSCK.value(False)

        self.RATE = 1  # 1 = 10Hz, 3 = 40Hz... extra clock cycles at the end

        # create the state machine
        self.sm = rp2.StateMachine(0, self.hx710c_pio, freq=100_000,
                                   sideset_base=self.pSCK, in_base=self.pOUT,
                                   jmp_pin=self.pOUT)
        self.set_rate(10);


    @rp2.asm_pio(
        sideset_init=(rp2.PIO.OUT_LOW, ), #*3,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def hx710c_pio():

        wait(0, pin, 0)     .side (0)   # wait for the device being ready
        wait(0, pin, 1)     .side (0)   # wait for the device being ready

        set(x, 7)
        label("bitloop1")
        nop()               .side (1) #(3)   # active edge
        in_(pins, 2)        .side (1) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop1")  .side (0) [1]  # test for more bits
        push(block)         #.side(4)   # no, deliver data and start over

        set(x, 7)
        label("bitloop2")
        nop()               .side (1) #(3)   # active edge
        in_(pins, 2)        .side (1) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop2")  .side (0) [1] # test for more bits
        push(block)            # no, deliver data and start over

        set(x, 7)
        label("bitloop3")
        nop()               .side (1) #(3)   # active edge
        in_(pins, 2)        .side (1) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop3")  .side (0) [1]  # test for more bits
        push(block)         .side (0)   # no, deliver data and start over

        #set(x, 0)
        #label("bitloop3")
        nop()               .side (1) [1] #(3) [1]   # active edge
        nop()               .side (0) [1]   # active edge

        label("finish")


    def set_rate(self, rate):
        if rate is 10:
            self.RATE = 1
        elif rate is 40:
            self.RATE = 3
        self.read()

    def is_ready(self):
        return self.pOUT() == 0

    def odd_bits(self, r):
        ch1 = r & 0x5555
        ch1 = (ch1 | ch1 >> 1) & 0x333333
        ch1 = (ch1 | ch1 >> 2) & 0x0f0f0f
        ch1 = (ch1 | ch1 >> 4) & 0xff
        return ch1

    def even_bits(self, r):
        ch1 = r & 0xAAAA
        ch1 = ch1 >> 1
        return self.odd_bits(ch1)
    def read2(self):
        self.sm.active(1)
        i = 0
        while True:
            print(i, hex(self.sm.get()))
            i += 1
            
    def read(self):
        # Feed the waiting state machine & get the data
        self.sm.active(1)  # start the state machine
#         self.sm.put(250_000)     # set wait time to 500ms
#         self.sm.put(self.RATE + 24 - 1)     # set pulse count 25-27, start
        result = 0;
        result2 = 0;
        r = self.sm.get();
        ch1 = self.odd_bits(r)
        ch2 = self.even_bits(r)
        # print(hex(ch1), hex(ch2))
        result += (ch1<<16)
        result2 += (ch2<<16)
        r = self.sm.get();
        ch1 = self.odd_bits(r)
        ch2 = self.even_bits(r)
        # print(hex(ch1), hex(ch2))
        result += (ch1<<8);
        result2 += (ch2<<8)
        r = self.sm.get() # >> (2*self.RATE);        
        ch1 = self.odd_bits(r)
        ch2 = self.even_bits(r)
        # print(hex(ch1), hex(ch2))
        result += ch1 # get the result & discard RATE bits
        result2 += ch2
        # result = self.sm.get() >> self.RATE # get the result & discard RATE bits
        # print(hex(result))
        self.sm.active(0)  # stop the state machine
        if result == 0x7fffffff:
            raise OSError("Sensor does not respond")

        # check sign
        if result > 0x7fffff:
            result -= 0x1000000

        # check sign
        if result2 > 0x7fffff:
            result2 -= 0x1000000

        return result, result2

    def read_average(self, times=3):
        print('read_average')
        sum = 0
        for i in range(times):
            sum += self.read()
        return sum / times

    def power_down(self):
        self.pSCK.value(False)
        self.pSCK.value(True)

    def power_up(self):
        self.pSCK.value(False)

ONCE = False
CONTINUOUS = False

def readUSBSerial():
    global ONCE, CONTINUOUS, last
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
    msg += '\n'
    uart.write(msg)
    
def readSerial():
    global ONCE, CONTINUOUS, last
    gc.collect()
    events = poll.poll(0)
    for event in events:
        #print('Got USB serial message')
        if event[0]==uart:
            gc.collect()
            #print('in uart poll')
            cmd = uart.readline()
            #print(type(cmd), repr(cmd))
            cmd = cmd.strip().upper()
            if type(cmd)==bytes:
                cmd = cmd.decode()
            doCommand(cmd, uartresponder)

def doCommand(cmd, responder=print):
    global ONCE, CONTINUOUS, last
    #print('do Command', cmd)
    if len(cmd):  # respond to command
        if (cmd[0]=='Q'):
            responder('Got Q')
            exit(0)
        elif (cmd=='PING'):
            responder('PONG')
        elif (cmd=="R?"):
            #print(type(last), last)
            responder(f'{last}')
            #ONCE = True
        elif (cmd=="C"):
            CONTINUOUS = not CONTINUOUS
            responder(f"C: {CONTINUOUS}")
        elif (cmd=="C?"):
            responder(f'{CONTINUOUS}')
print('name', __name__)
if (__name__ == 'lockin_v2') or (__name__=='__main__'):
#if True:
    '''
    p0 = Pin(0, Pin.IN)  # spi MISO
    p3 = Pin(3, Pin.OUT)   # spi MOSI
    '''
    #p2 = Pin(2, Pin.OUT)  # spi MISO
    #p3 = Pin(3, Pin.OUT)
    p4 = Pin(4, Pin.OUT)   # spi MOSI
    in1 = Pin(2, Pin.IN)  # spi MISO
    in2 = Pin(3, Pin.IN)
    
    p6 = Pin(26, Pin.OUT) # high = bias on
    p7 = Pin(27, Pin.OUT) # low = low bias current
    p8 = Pin(28, Pin.OUT) # low = low bias current

    p6.on()  # turn on bias
    p7.on() # use low bias range
    p8.on()
    adc = HX710C(p4, in1)
    #adc.read2()
    N = 5
    def readN(N):    
        for count in range(5):
            raw = adc.read()
        r100k = 0
        rt = 0
        for count in range(N):
            raw = adc.read()
            reading = (raw[0]/ (1<<24) * 2.5 / 128 * 1e6,
                       raw[1]/ (1<<24) * 2.5 / 128 * 1e6,
                       )
            #print(reading)
            r100k += reading[0]
            rt += reading[1]
        return r100k, rt
    outer = [0,0]
    prev_outer = [0,0]
    ewa = 0
    alpha = 0.25
    counter = 0
    while True:
        #print(gc.mem_free())
        readUSBSerial()
        readSerial()
        #print('6')
        p6.toggle()
        #print(hex(raw[0]), hex(raw[1]))
        for i in range(1):
            prev = [0, 0]
            p7.toggle()
            p8.toggle()
            switches = (p6.value()<<2) + (p7.value()<<1) + p8.value()
            r100k, rt = readN(N)
            #print('%d %8.2f %8.2f %8.2f %8.2f %2.3f'%(switches, r100k, rt, r100k-prev[0], rt-prev[1], (r100k-prev[0])/(rt-prev[1]) ) )
            prev = [r100k, rt]
            p7.toggle()
            p8.toggle()
            readUSBSerial()
            readSerial()
            r100k, rt = readN(N)
            switches = (p6.value()<<2) + (p7.value()<<1) + p8.value()
            #print('%d %8.2f %8.2f %8.2f %8.2f %2.3f'%(switches, r100k, rt, r100k-prev[0], rt-prev[1], (r100k-prev[0])/(rt-prev[1]) ) )
        outer = [r100k, rt]
        V100K = outer[0]-prev_outer[0]
        cm0 = outer[0] + prev_outer[0]
        VS = outer[1] - prev_outer[1]
        cm1 = outer[1] + prev_outer[1]
        ewa = alpha * VS/V100K * 100 + (1-alpha)*ewa
        last = [time.time(), counter, VS/V100K*100, ewa]
        #print('outer',switches, outer, V100K, VS, VS/V100K*100, ewa, end='\r') #, cm0, cm1)
        if ONCE | CONTINUOUS:
            print(f'{counter} {time.time()} {switches} {V100K:.6} {VS:.6} {VS/V100K*100:.6} {ewa:.6} {rtc.datetime()}')
            ONCE = False
        prev_outer = outer
        counter += 1
