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
pwmServo.duty_ns(1500000)
pwmMotor.duty_ns(1565000)
time.sleep(0.5)
pwmMotor.duty_ns(1573000)
time.sleep(0.5)


#pwmServo.duty_ns(1500000)
print("Motor test done")
led.on()


while True:
    # Wait for data from the Raspberry Pi 4
    if uart.any():
        data = uart.read(7)
        if data:
            try:
                char = data.decode('utf-8')
                print("Received from Pi 4:", char)
                pw = int(char)
                if pw >= 1100000 and pw <= 1900000: #pwm is the speed; in_a/in_b is the direction
                    #if pw >= 1500000:
                    # set the value low then high
                    if pw >= 1300000 and pw <= 1700000:
                        pwmServo.duty_ns(pw)
                        pwmMotor.duty_ns(1579000)
                        #pwmMotor.duty_ns(1577000)
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())

                        #pwmMotor.duty_ns(1500000)
                        print("forward")
                    elif pw > 1700000 and pw <= 1900000:
                        pwmMotor.duty_ns(1572000)
                        pwmServo.duty_ns(pw)
                        #pwmMotor.duty_ns(1577000)
                        #pwm = 1577000 * 0.8
                        #pwmMotor.duty_ns(int(pwm))
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())
                        #pwmServo.duty_ns(pw)
                        #pwmMotor.duty_ns(1500000)
                        print("left")
                    elif pw >= 1100000 and pw < 1300000:
                        pwmServo.duty_ns(pw)
                        pwmMotor.duty_ns(1572000)
                        #pwmMotor.duty_ns(1577000)
                        #pwm = 1577000 * 0.8
                        #pwmMotor.duty_ns(int(pwm))
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())
                        #pwmServo.duty_ns(pw)
                        #pwmMotor.duty_ns(1500000)
                        print("right")
                    #pwmServo.duty_ns(pw)
                    #print("here")
            except:
                print("Error. Skipping")
                continue
    time.sleep(0.1)  # Short delay to avoid using too much CPU

else:
    print("error reading uart")

