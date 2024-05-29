#import libraries
import cv2 #cv2 is an openCV library
import numpy as np #allows numerical computations
from picamera2 import Picamera2 #rasPi libary -> allows for capturing images 7 videos
import time
import serial
import tty, sys, termios

print("Start")


import datetime

# Current timestamp
timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

# Initialize video writer with timestamp in filename
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_video = cv2.VideoWriter(f'output_{timestamp}.avi', fourcc, 20.0, (640, 480))  # Adjust resolution as necessary

prev_steer_angle = 0
prev_error = 0
integral_val = 0
x = "S"
#no_line_flag = 0

#init serial connection
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

ser.flush()

#mask the image to only show the ROI (reduces noise & distracting objects)
def region_of_interest(img, vertices):
    mask = np.zeros_like(img) #create a black image (mask) the same size as input image
    cv2.fillPoly(mask, vertices, 255) #fill specified ROI with white on created mask
    masked_img = cv2.bitwise_and(img, mask) #use bitwise btwn mask & orignial image to keep just ROI
    return masked_img #within the ROI, original values retained, and the rest of image will be black

def update_PID (setpoint, offset, dt):
    Kp = 0.1 #proportional gain
    Ki = 0.01 #integral gain
    Kd = 0.05 #derivative gain
    
    global prev_error, integral_val
    
    #calculate error
    error = setpoint - offset
    
    P_term = Kp * error
    
    integral_val += error * dt
    I_term = Ki * integral_val
    
    der = (error - prev_error) / dt
    D_term = Kd * der
    
    output = P_term + I_term + D_term
    
    prev_error = error
    
    return output
    
def get_steering_angle (angle, min_pwm, max_pwm, max_turn_angle):
    pw = ((angle + (max_turn_angle / 2)) / max_turn_angle) * (max_pwm - min_pwm) + min_pwm
    return int(pw)
    
#draw lines based on the hough line transform
def draw_lines(img, lines):
    x_values = []
    y_values = []
    global x
   # global no_line_flag
    if lines is not None:  #if there is a line
        #no_line_flag = 1
        for line in lines: #iterate through each line 
            for x1, y1, x2, y2 in line:
                if x2-x1 == 0:
                    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3) #BGR format
                    #print("Here")
                else:
                    slope = (y2-y1) / (x2-x1)
                    if np.abs(slope) > 1:
                        x_values += [x1, x2]
                        y_values += [y1, y2]
                        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3) #BGR format
                        #print ("x_values = ", x_values)
                        #print ("y_values = ", y_values)
                
                        #draw each line with a color green and a thickness of 3 pixels
                        #use green because it has greatest intensity (so it will be bright grey later)
        #draw center circle
        if len(x_values) > 0 and len(y_values) > 0:
            avg_x = int(np.mean(x_values))
            avg_y = int(np.mean(y_values))
    
            offset = (img.shape[1] // 2) - avg_x #343 - (320) = 23
            correction = update_PID(0, offset, 1/30)
            steer_angle = get_steering_angle(correction, 1100, 1900, 90)
            if (steer_angle < 1900 and steer_angle > 1100):
                cv2.circle(img, (avg_x,avg_y), 5, (255,0,0), -1)
                if steer_angle >= 1700 and x != "L":
                    print ("Turn left")
                    x = "L"
                    new_steer_angle = (abs(1900-steer_angle)+1100)*1000
                    ser.write(str(new_steer_angle).encode('utf-8'))
                    steer_angle = new_steer_angle
                    #time.sleep(2)
                elif steer_angle <= 1300 and x != "R":
                    print ("Turn right")
                    x = "R"
                    new_steer_angle = (abs(1900-steer_angle)+1100)*1000
                    ser.write(str(new_steer_angle).encode('utf-8'))
                    steer_angle = new_steer_angle
                    
                    #time.sleep(2)
                else:
                    if x != "F" :
                        print ("Going forward")
                        x = "F"
                        new_steer_angle = (abs(1900-steer_angle)+1100)*1000
                        ser.write(str(new_steer_angle).encode('utf-8'))
                        #time.sleep(2)
                        steer_angle = new_steer_angle
          
                print ("steer_angle = ", steer_angle)
               
        

def process_image(img, img2):
    #convert original image to greyscale (color not needed for edge detection)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #uses gaussian blur to filter noise
    #each pixel's new value = weighted avg of itself & neighbor values (weights decrease with distance from pixel)
    #lower std dev = more blur
    #stddev parameter = 0 means opencv will autocalulate the optimal stddev value for us
    blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0) #use5x5 kernel (5x5 grid of pixels for avging

    #use canny edge detection to find edges
    #uses thresholds to determine strong/weak/non-edge pixels
    #pixel gradient magnitude > threshold = strong
    #pgm < high but > low = weak
    #pgm < low = suppress pixel (make black)
    canny_img = cv2.Canny(blur_img, 100, 200)

    #define region of interest
    imshape = img.shape #get dimensions of our image (imshape[0] = height, imshape[1] = width)
    #bottom left, top left, top right, bottom right (each are ordered pairs)
    #vertices = np.array([[(0, imshape[0]), (450, 320), (490, 320), (imshape[1], imshape[0])]], dtype=np.int32)
    vertices = np.array([[(-100, imshape[0]), (150, imshape[0]*0.25), (500, imshape[0]*0.25), (imshape[1]+100, imshape[0])]], dtype=np.int32)

    masked_img = region_of_interest(canny_img, vertices)

    #probabilistic hough line transform to detect lines
    #use image after applying ROI mask
    lines = cv2.HoughLinesP(masked_img, 1, np.pi/180, 50, np.array([]), minLineLength=50, maxLineGap=10)

    #create an empty black image to draw lines on (3 color channels, w/ image pixels being 8-bit unsigned ints)
    #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
    #draw lines detected from hough transform onto blank img in ROI
    draw_lines(line_img, lines)
    draw_lines(img2, lines)
    
    
    #combine original image with the line image to get the clear, non-noisy final
    final_img = cv2.addWeighted(img, 0.8, line_img, 1, 0)

    return final_img, masked_img, line_img

picam2 = Picamera2()
picam2.start()
print("Camera started, press 'q' to quit.")

while True:
    try:
        array = picam2.capture_array("main")
        frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        frame2 = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

        final_img, masked_img, line_img = process_image(frame, frame2)

        # Write the frame to video
        output_video.write(final_img)  # You might choose to record 'final_img', 'frame', or another processed frame

        time.sleep(0.1)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
           break
    except KeyboardInterrupt:
        break

# Cleanup
cv2.destroyAllWindows()
picam2.close()
ser.close()
output_video.release()  # Release the video writer

print('Stopped')
