import machine
from machine import Pin
import time
from time import sleep
from machine import PWM
from machine import I2S

   

frequency = 1000
interrupt = 0
start = time.ticks_ms()
end_time = 0
interval = 0
timevals = []
value = 0
value_min = 0
values = [0,0,0,0]
def handle_interrupt(pin):
    global start
    global interval
    global timevals
    global value
    global value_min
    interval = time.ticks_diff(time.ticks_ms(), start)
    #print(interval)
    timevals.append(interval)
    start = time.ticks_ms()
    global interrupt
    interrupt += 1
    if len(timevals)>= 3:
        value = 3/(timevals[-1] + timevals[-2] + timevals[-3])*1000000
    ss = 0
    for i in range(0,len(timevals)):
        ss = ss + timevals[i]
    if ss >= 60000:
        sss = 0
        k = 1
        while sss <= 60000:
            sss = sss + timevals[-k]
            k = k + 1
        value_min = (k-1)/sss*1000000
    beep(100)
    values[3] = int((value//1)%10)
    values[2] = int((value//10)%10)
    values[1] = int((value//100)%10)
    values[0] = int((value//1000)%10)
    print('omaewa ', values, value_min, interrupt, timevals)
    

def beep(duration_ms):
    pwm.freq(frequency)
    pwm.duty_u16(32768) 
    sleep(duration_ms/1000)
    pwm.duty_u16(0)



led = Pin(26, Pin.OUT)
pir = Pin(26, Pin.IN)

D1 = Pin(21, Pin.OUT)
D2 = Pin(22, Pin.OUT)
D3 = Pin(23, Pin.OUT)
D4 = Pin(25, Pin.OUT)

A = Pin(15, Pin.OUT)
B = Pin(2, Pin.OUT)
C = Pin(4, Pin.OUT)
D = Pin(16, Pin.OUT)
E = Pin(17, Pin.OUT)
F = Pin(5, Pin.OUT)
G = Pin(18, Pin.OUT)
DP = Pin(19, Pin.OUT)

sound_pin = Pin(27)

pwm = PWM(sound_pin)

pir.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)






while True:
#     values[3] = (value//1)%10
#     values[2] = (value//10)%10
#     values[1] = (value//100)%10
#     values[0] = (value//1000)%10
    number_patterns = [
        [0, 0, 0, 0, 0, 0, 1],  # 0
        [1, 1, 1, 1, 0, 0, 1],  # 1
        [0, 0, 1, 0, 0, 1, 0],  # 2
        [0, 0, 0, 0, 1, 1, 0],  # 3
        [1, 0, 0, 1, 1, 0, 0],  # 4
        [0, 1, 0, 0, 1, 0, 0],  # 5
        [0, 1, 0, 0, 0, 0, 0],  # 6
        [0, 0, 0, 1, 1, 1, 1],  # 7
        [0, 0, 0, 0, 0, 0, 0],  # 8
        [0, 0, 0, 1, 1, 0, 0]   # 9
    ]
    common_pins = [
        [1, 0, 0, 0],  # Включение разряда 1
        [0, 1, 0, 0],  # Включение разряда 2
        [0, 0, 1, 0],  # Включение разряда 3
        [0, 0, 0, 1]   # Включение разряда 4
    ]

    for n in range(4):
        
        
        
        D1.value(common_pins[n][0])
        D2.value(common_pins[n][1])
        D3.value(common_pins[n][2])
#        D4.value(common_pins[n][3])
        
        A.value(number_patterns[values[n]][0])
        B.value(number_patterns[values[n]][1])
        C.value(number_patterns[values[n]][2])
        D.value(number_patterns[values[n]][3])
        E.value(number_patterns[values[n]][4])
        F.value(number_patterns[values[n]][5])
        G.value(number_patterns[values[n]][6])
        sleep(0.005)    

 
