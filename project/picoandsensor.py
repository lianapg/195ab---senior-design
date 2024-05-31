from machine import Pin, UART, PWM, time_pulse_us
import utime
import time

# Define the HC-SR04 sensor pins
trigger = Pin(3, Pin.OUT)
echo = Pin(1, Pin.IN)

# Define the distance measurement function
def get_distance():
    # Ensure trigger is low
    trigger.low()
    utime.sleep_us(2)
    
    # Send a 10us pulse to trigger
    trigger.high()
    utime.sleep_us(10)
    trigger.low()
    
    # Measure the duration of the echo pulse
    duration = time_pulse_us(echo, 1, 30000)  # Wait for a high pulse with a timeout of 30000us
    
    if duration < 0:
        return -1  # Timeout
    
    # Calculate the distance (duration is in microseconds)
    distance = (duration * 0.0343) / 2
    
    return distance

# Set up UART and PWM
led = Pin("LED", Pin.OUT)
uart = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
uart.init(bits=8, parity=None, stop=1)

pwmServo = PWM(Pin(0)) #ch 2
pwmMotor = PWM(Pin(2)) #ch 1, changed from Pin 2 to Pin 1

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

print("Motor test done")
led.on()

while True:
    # Measure distance
    distance = get_distance()
    if distance != -1:
        print("Distance: {:.2f} cm".format(distance))
    
    # Speed up the car if an object is detected within 5 cm
    if distance != -1 and distance < 5:
        pwmMotor.duty_ns(1600000)  # Speed up the motor
        print("Object detected within 5 cm! Speeding up the motor.")
    else:
        pwmMotor.duty_ns(1573000)  # Normal speed

    # Wait for data from the Raspberry Pi 4
    if uart.any():
        data = uart.read(7)
        if data:
            try:
                char = data.decode('utf-8')
                print("Received from Pi 4:", char)
                pw = int(char)
                if pw >= 1100000 and pw <= 1900000: #pwm is the speed; in_a/in_b is the direction
                    if pw >= 1300000 and pw <= 1700000:
                        pwmServo.duty_ns(pw)
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())
                        print("forward")
                    elif pw > 1700000 and pw <= 1900000:
                        pwmMotor.duty_ns(1572000)
                        pwmServo.duty_ns(pw)
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())
                        print("left")
                    elif pw >= 1100000 and pw < 1300000:
                        pwmServo.duty_ns(pw)
                        pwmMotor.duty_ns(1572000)
                        print('pwm_servo=', pwmServo.duty_ns())
                        print('pwm_motor=', pwmMotor.duty_ns())
                        print("right")
            except:
                print("Error. Skipping")
                continue
    time.sleep(0.1)  # Short delay to avoid using too much CPU

else:
    print("error reading uart")
