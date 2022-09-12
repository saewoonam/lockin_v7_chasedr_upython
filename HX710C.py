from machine import Pin, idle, UART, RTC, enable_irq, disable_irq
import time
import rp2
from ulab import numpy as np
import random

# adapted from https://github.com/robert-hh/hx711/blob/master/hx711_pio.py
class HX710C:
    def __init__(self, clk, data):
        '''
            will use 25 clock cycles so that the rate is 10Hz
            clk and data are pin numbeers
        '''
        self.clk = clk
        self.data = data
        self.reset_sm()
        
    def reset_sm(self):
        self.pSCK = Pin(self.clk, Pin.OUT)
        self.pOUT = Pin(self.data)
        self.pSCK.value(False)
        
        # create the state machine
        self.sm = rp2.StateMachine(0, self.hx710c_pio, freq=500_000,
                                   sideset_base=self.pSCK, in_base=self.pOUT,
                                   jmp_pin=self.pOUT)

    @rp2.asm_pio(
        sideset_init=(rp2.PIO.OUT_LOW, ), #*3,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def hx710c_pio():
        # uses sideset to generate the clock for the ADC
        #   ADC clock = 6 PIO clock cycles
        #   clock data into PIO on the down transition of ADC clock to deal with propagation delays in slow isolators
        #      three nop for high ADC clock high
        #      one clock cycle to read in and two left for jmp with wait state or push + set(x)
        #   unroll the last bit from the loop in x so that we can have a symmetric clock
        wait(0, pin, 0)     .side (0)   # wait for the device being ready
        wait(0, pin, 1)     .side (0)   # wait for the device being ready

        set(x, 6)
        label("bitloop1")
        nop()               .side (1) [2]
        in_(pins, 2)        .side (0) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop1")  .side (0) [1]  # test for more bits
        nop()               .side(1) [2]
        in_(pins,2)         .side(0)   
        push(block)         .side(0)#.side(4)   # no, deliver data and start over
     
        set(x, 6)
        label("bitloop2")
        nop()               .side (1) [2] #(3)   # active edge
        in_(pins, 2)        .side (0) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop2")  .side (0) [1] # test for more bits
        nop()               .side(1) [2]
        in_(pins,2)         .side(0)
        push(block)         .side(0)   # no, deliver data and start over

        set(x, 7)
        label("bitloop3")
        nop()               .side (1) [2]#(3)   # active edge
        in_(pins, 2)        .side (0) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop3")  .side (0) [1]  # test for more bits
        # 25th clock cycle
        push(block)         .side (1)   # no, deliver data and start over
        nop()               .side (1) [1] #(3) [1]   # active edge
        nop()               .side (0) [2]   # active edge

    @rp2.asm_pio(
        sideset_init=(rp2.PIO.OUT_LOW, ), #*3,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def hx710c_pio_loop():

        #wait(0, pin, 0)     .side (0)   # wait for the device being ready
        wait(0, pin, 1)     .side (0)   # wait for the device being ready
        wait(0, pin, 0)     .side (0)   # wait for the device being ready
        
        set(y, 2)
        label("byteloop")
        set(x, 7)
        label("bitloop1")
        nop()               .side (1) #(3)   # active edge
        in_(pins, 2)        .side (1) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop1")  .side (0) [1]  # test for more bits
        push(block)         #.side(4)   # no, deliver data and start over
        jmp(y_dec, "byteloop")

        nop()               .side (1) [1] #(3) [1]   # active edge
        nop()               .side (0) [1]   # active edge

        label("finish")

    @rp2.asm_pio(
        sideset_init=(rp2.PIO.OUT_LOW, ), #*3,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def hx710c_pio_loop_v2():

        #wait(0, pin, 0)     .side (0)   # wait for the device being ready
        wait(0, pin, 1)     .side (0)   # wait for the device being ready
        wait(0, pin, 0)     .side (0)   # wait for the device being ready
        
        set(y, 2)  # Get 1 byte from each ADC in each loop for 24 bits
        label("byteloop")
        set(x, 6)           .side(0)  # get first 7 bits,  ADC clock cycle is 6 state machine steps
        label("bitloop1")
        nop()               .side (1) [1]#(3)   # clock edge high
        in_(pins, 2)        .side (1) #(3)   # get the pin and shift it in
        jmp(x_dec, "bitloop1")  .side (0) [2]  # test for more bits
        nop()               .side (1) [1] #(3)   # clock edge for 8th bit
        in_(pins, 2)        .side (1) #(3)   # get 8th bit
        push(block)         .side(0) #.side(4)   # push 16bits (8 from each ADC)
        jmp(y_dec, "byteloop") .side(0)
        nop()               .side(0)
        nop()               .side (1) [2] # 25th clock cycle
        nop()               .side (0) [2] 
        
        
    def odd_bits(self, r):
        ch1 = r & 0x5555
        ch1 = (ch1 | ch1 >> 1) & 0x333333
        ch1 = (ch1 | ch1 >> 2) & 0x0f0f0f
        ch1 = (ch1 | ch1 >> 4) & 0xff
        return ch1

    def even_bits(self, r):
        # convert number so all the bits are in the odd positions then call odd_bits to retrieve the number
        ch1 = r & 0xAAAA
        ch1 = ch1 >> 1
        return self.odd_bits(ch1)

    def read(self):
        #self.sm.active(1)  # start the state machine
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
        r = self.sm.get()
        ch1 = self.odd_bits(r)
        ch2 = self.even_bits(r)
        # print(hex(ch1), hex(ch2))
        result += ch1  
        result2 += ch2
        # print(hex(result))
        #self.sm.active(0)  # stop the state machine
        if result == 0x7fffffff:
            raise OSError("Sensor does not respond")

        # check sign
        if result > 0x7fffff:
            result -= 0x1000000

        # check sign
        if result2 > 0x7fffff:
            result2 -= 0x1000000

        return result, result2

print('name', __name__)
if (__name__=='__main__'):
    p6 = Pin(26, Pin.OUT) # high = bias on
    p7 = Pin(27, Pin.OUT) # low = low bias current
    p8 = Pin(28, Pin.OUT) # low = low bias current
    p9 = Pin(29, Pin.OUT)
    
    p6.on()  # turn on bias
    p7.on() # use low bias range
    p8.on()
    p9.off()
    
    adc = HX710C(2, 6)
    conversion = 1e6 * 2.5 / (1<<24) / 128  # converts to uV

    def test_read_both():
        # prints output of digitizers... overwrites when values look fine
        # prints extra line if data looks funny
        prev=None
        counter=0
        adc.sm.active(1)
        while True:
            data = adc.read()
            t = time.ticks_ms()
            if prev is not None:
                if (abs(data[0]-prev[0]) > 10000) or ((t-prev_t)>100) or (abs(data[1]-prev[1])>10000):
                    print(f'{counter:>6} {t-prev_t:>4} {data[0]:>9} {data[1]:>9} {data[0]-prev[0]:>9} {data[1]-prev[1]:>9} {t}')
                else:
                    print(f'{counter} {t-prev_t} {data[0]:>9} {data[1]:>9}', end='\r')
            counter += 1        
            prev = data
            prev_t = t
            #print(time.ticks_ms(), adc.read())
    #test_read_both()
    def read_only():
        prev=None
        counter = 0
        adc.sm.active(1)
        while True:
            data = adc.read()
            t = time.ticks_ms()
            if prev is not None:
                print(f'{counter} {t-prev_t} {data[0]:>9} {data[1]:>9}')
            counter +=1
            prev = data
            prev_t = t;
    #read_only()
    def read_random():
        prev=None
        counter = 0
        adc.sm.active(1)
        while True:
            switches = random.randint(0,7)
            p6.on() if (switches & 4) else p6.off()
            p7.on() if (switches & 2) else p7.off()
            p8.on() if (switches & 1) else p8.off()

            for count in range(5):
                adc.read()
            data = adc.read()
            t = time.ticks_ms()
            if prev is not None:
                print(f'{counter} {t-prev_t} {switches} {data[0]:>9} {data[1]:>9}')
            counter +=1
            prev = data
            prev_t = t;
    #read_random()
    def reset():
        p2 = Pin(2, Pin.OUT)
        p2.value(0)
        p2.value(1)
        time.sleep(0.001)
        p2.value(0)
        adc.reset_sm()
    def reset_on_toggle():
        # no funny data observed... 17000 + 
        prev=None
        counter = 0
        avg = np.array([0, 0])
        while True:
            data = adc.read()
            t = time.ticks_ms()
            #print(t, data)
            if prev is not None:
                if ((abs(data[0]-prev[0]) > 10000) or (abs(data[1]-prev[1])>10000)) and ((counter%10)!=1):
                    print()
                    print(counter%10, (counter%10)!=1)
                    print(f'{t-prev_t:>4} {data[0]:>9} {data[1]:>9} {data[0]-prev[0]:>9} {data[1]-prev[1]:>9} {counter:>6} {t}')
                else:
                    print(f'{counter} {t-prev_t} {data[0]:>9} {data[1]:>9}', end='\r')
                    avg += list(data)
            else:
                avg = np.array(list(data))
            if(counter%10==0):
                p6.toggle()
                reset()
                prev=None
            counter += 1        
            prev = data
            prev_t = t
    #reset_on_toggle()            
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
    def toggle6():
        counter = 0
        adc.sm.active(1);
        p7.off()
        p8.off()
        while True:
            p6.toggle()
            readings = readN_reset(5)
            new_readings = np.mean(readings, axis=0)
            if counter>0:
                diff = new_readings-prev_readings
                diff /= 2.0
                offset = new_readings + prev_readings
                offset /=2
                #print(f'{diff[0]:9.7} {diff[1]:9.7} {offset[0]:>9.6} {offset[1]:>9.7}') # {new_readings}')
                
                r = diff[1]/diff[0] * 100
                alpha = 0.05
                if counter == 1:
                    ewa = r;
                ewa = r*alpha + (1-alpha)*ewa
                print(f'{r:6.4} {ewa:6.4} {offset[0]:>9.5} {offset[1]:>9.7}')
                
            prev_readings = new_readings
            counter += 1
    #toggle6()
    #p7.toggle()
    def toggle8():
        counter = 0
        adc.sm.active(1);
        while True:
            p8.toggle()
            readings = readN_reset(5)
            new_readings = np.mean(readings, axis=0)
            if counter>0:
                diff = new_readings-prev_readings
                diff /= 2.0
                offset = new_readings + prev_readings
                offset /=2
                print(f'{diff[0]:9.7} {diff[1]:9.7} {offset[0]:>9.6} {offset[1]:>9.7}') # {new_readings}')
                
            prev_readings = new_readings
            counter += 1
    #toggle8()
    #p7.toggle()
    def toggle678():
        counter = 0
        adc.sm.active(1)
        toggle_bias = True
        while True:
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
                alpha = 0.20
                if counter == 1:
                    ewa = r;
                if (abs(offset[0])>8000):
                    # Don't update ewa if the offset is off.
                    #print(f'{counter} {r:6.4} {ewa:6.4} {offset[0]:>6.3} {offset[1]:>6.5} {inner_offset[0]:>6.3} {inner_offset[1]:>6.5}')
                    print(f'{counter} {r:6.2f} {ewa:6.2f} {offset[0]:>9.2f} {offset[1]:>9.2f} {inner_offset[0]:>9.2f} {inner_offset[1]:>9.2f}')
                    print(r1, r2)
                    print(np.mean(r1, axis=0), np.mean(r2, axis=0))
                    toggle_bias = False
                    current_readings = prev_readings # setup to try again
                else:
                    ewa = r*alpha + (1-alpha)*ewa                
                    #print(f'{counter} {r:6.4} {ewa:6.4} {offset[0]:>6.3} {offset[1]:>6.5} {inner_offset[0]:>6.3} {inner_offset[1]:>6.5}', end='\r')
                    print(f'{counter} {r:6.4f} {ewa:6.4f} {offset[0]:>9.2f} {offset[1]:>9.2f} {inner_offset[0]:>9.2f} {inner_offset[1]:>9.2f}', end='\r')
                    toggle_bias = True
            #print(current_readings)
            prev_readings = current_readings
            counter += 1
    toggle678()
