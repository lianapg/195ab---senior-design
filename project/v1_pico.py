from machine import Pin,UART
from machine import PWM
import time

led = Pin("LED", Pin.OUT)
#led.on()
uart = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
uart.init(bits=8, parity=None, stop=1)

pwmServo = PWM(Pin(0)) #ch 2
pwmMotor = PWM(Pin(2)) #ch 1

pwmServo.freq(100)
pwmMotor.freq(100)

print("Pico UART initialized, waiting for data...")
pwmMotor.duty_ns(1500000)
pwmServo.duty_ns(1500000)
time.sleep(0.5)
pwmMotor.duty_ns(1520000)
pwmServo.duty_ns(1700000)
time.sleep(0.5)
pwmMotor.duty_ns(1540000)
pwmServo.duty_ns(1500000)
time.sleep(0.5)
pwmMotor.duty_ns(1560000)
pwmServo.duty_ns(1300000)
time.sleep(0.5)
pwmMotor.duty_ns(1565000)
time.sleep(0.5)
pwmMotor.duty_ns(1570000)
time.sleep(0.5)


#pwmServo.duty_ns(1500000)
print("Motor test done")
led.on()


while True:
    # Wait for data from the Raspberry Pi 4
    if uart.any():
        data = uart.read(1)
        if data:
            char = data.decode('utf-8')
            print("Received from Pi 4:", char)

            if char=="F":
                led.toggle()
                print("Forward")
                #pwmMotor.duty_ns(1550000) 
                pwmMotor.duty_ns(1577000)
                pwmServo.duty_ns(1500000)
                #time.sleep(2)  # Short delay to avoid using too much CPU
            elif char=="L":
                led.toggle()
                print("Left")
                #pwmMotor.duty_ns(1550000) 
                pwmMotor.duty_ns(1577000)
                pwmServo.duty_ns(1200000)
                #time.sleep(2)  # Short delay to avoid using too much CPU
            #elif char=="s":
               # print("Brake")
                #pwmMotor.duty_ns(1500000) 
               # pwmServo.duty_ns(1500000) 
            elif char=="R":
                led.toggle()
                print("Right")
                #pwmMotor.duty_ns(1550000)
                pwmMotor.duty_ns(1577000)
                pwmServo.duty_ns(1800000)

                #time.sleep(0.1)  # Short delay to avoid using too much CPU
           # elif char=="x":
             #   print("Backward")
             #   pwmMotor.duty_ns(1500000) 
             #   pwmMotor.duty_ns(1400000) 
             #   pwmServo.duty_ns(1500000)
            #else:
                #pwmMotor.duty_ns(1500000)
                #pwmServo.duty_ns(1500000)
    time.sleep(0.1)  # Short delay to avoid using too much CPU

else:
    print("error reading uart")

