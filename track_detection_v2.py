# Single Color Grayscale Blob Tracking Example
#
# This example shows off single color grayscale tracking using the OpenMV Cam.

import sensor, image, time, math
from pyb import LED
import time
from pyb import Pin, Timer


tim_motor = Timer(4, freq=2000) # Frequency in Hz
tim_servo = Timer(2, freq=300)

#set in1 and in2 to 1 and 0 for now, later need to edit:
inA = Pin("P0", Pin.OUT_PP)
inB = Pin("P1", Pin.OUT_PP)
inA.high()
inB.low()
found_finish_line = False

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (175, 255) #250 255 for ball room

# roi is x, y width, height, weight
ROIS = [ # [ROI, weight]
        #(0, 100, 160, 10, 0.7), # You'll need to tweak the weights for your app
        (0,  50, 160, 15, 0.7), # depending on how your robot is setup.
        (40, 40, 25, 15, 0.7),
        (80, 40, 25, 15, 0.7),
        (35, 55, 25, 15, 0.7), # find finish line on the left
        (85, 55, 25, 15, 0.7)
        #(0,  50, 160, 10, 0.7)
       ]
weight_sum = 0

#pulse_width for motor control:
motor_pulse_percent = 0
servo_pulse_percent = 0

for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
loop_count = 0
run_count = 0
previous_center_x = 0
pp_center_x = 0
speed_index = 0.65
while(True):
    #if loop_count <= 7000:
        #speed_index = 0.4
    #else:
        #speed_index = 0.75
    loop_count = loop_count + 1
    clock.tick()
    img = sensor.snapshot()
    centroid_sum = 0
    x_cord = [0,0]
    y_cord = [0,0]
    center_x = 0
    found_line = False
    finish_blob = 0

    for r in ROIS:
        if r == ROIS[0]:
            blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 100, area_threshold= 100, merge=True) # r[0:4] is roi tuple.
        else:
            blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 50, area_threshold= 50, merge=True) # r[0:4] is roi tuple.
        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())

            ## Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect(),color=0)
            img.draw_cross(largest_blob.cx(),largest_blob.cy(),color = 0)
            #img.draw_cross(60,50,color = 0)
            #img.draw_cross(100,50,color = 0)
            #img.draw_cross(55,70,color = 0)
            #img.draw_cross(105,70,color = 0)
            img.draw_rectangle(40, 40, 25, 15, color=10)
            img.draw_rectangle(80, 40, 25, 15, color=10)
            img.draw_rectangle(35, 55, 25, 15, color=10)
            img.draw_rectangle(85, 55, 25, 15, color=10)
            if r == ROIS[0]:
                if blobs:
                    center_x = largest_blob.cx()
                    found_line = True
                else:
                    found_line = False
            if r == ROIS[1]:
                if blobs:
                    print('upper left found')
                    finish_blob = finish_blob + 1
            if r == ROIS[2]:
                if blobs:
                    print('upper right found')
                    finish_blob = finish_blob + 1
            if r == ROIS[3]:
                if blobs:
                    print('lower left found')
                    finish_blob = finish_blob + 1
            if r == ROIS[4]:
                if blobs:
                    print('lower right found')
                    finish_blob = finish_blob + 1
    center_off = previous_center_x - 80 #p controller
    d_off = (center_x - pp_center_x) * clock.fps() / 2 #d controller
    #control algorithm:
    pidx = center_x + center_off * 0.0 + d_off * 0.000
    if found_line:
        if pidx < 30:
            print('at right')
            red_led.on()
            green_led.off()
            blue_led.off()
            #motor_pulse_percent = 50 #50 for normal value set to 0 for debug
            #servo_pulse_percent = 53
        elif pidx > 120:
            print('at left')
            blue_led.on()
            green_led.off()
            red_led.off()
            #motor_pulse_percent = 50
            #servo_pulse_percent = 37
        elif pidx <= 95 and pidx >= 55:
            print('at center')
            green_led.on()
            blue_led.off()
            red_led.off()
            # motor_pulse_percent = 80
            # servo_pulse_percent = 45
        elif pidx < 55 and pidx >= 40:
            print('near right')
            # servo_pulse_percent = 49
            # motor_pulse_percent = 60
            red_led.on()
            green_led.on()
            blue_led.off()
        elif pidx > 95 and pidx <= 110:
            print('near left')
            red_led.off()
            blue_led.on()
            green_led.on()

        motor_pulse_percent = -7.22222222 * 10**-2 * pidx**2 +  1.08333333 * 10 * pidx - 3.11249999 * 10**2
        servo_pulse_percent =   -5.389750725* 10**-5 * pidx**3 +   1.21269391315* 10**-2 * pidx**2 - 1.0094665373* 10**0 * pidx +  7.52339685 * 10**1
        if motor_pulse_percent <= 40:
            motor_pulse_percent = 40
        if servo_pulse_percent >= 55:
            servo_pulse_percent = 55
        if servo_pulse_percent <= 35:
            servo_pulse_percent = 35


    else:
        print('nothing detected')
        green_led.on()
        red_led.on()
        blue_led.on()
        motor_pulse_percent = 50
    #if finish_blob >= 3:
        #print('finish line found')
        #inA.low()
        #inB.low()
        #found_finish_line = True
    if found_finish_line:
        green_led.on()
        red_led.on()
        blue_led.on()
        time.sleep(100)
        green_led.off()
        red_led.off()
        blue_led.off()
        time.sleep(100)



    # for testing purpose, reduce the motor pulse percent so that the car runs slowly:
    motor_pulse_percent = motor_pulse_percent * speed_index
    ch1 = tim_motor.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=motor_pulse_percent)
    ch2 = tim_servo.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=servo_pulse_percent)


    #calculate the difference of two center points
    dx = x_cord[0] - x_cord[1]
    dy = y_cord[0] - y_cord[1]


    deflection_angle = 0



#=======
## Single Color Grayscale Blob Tracking Example
##
## This example shows off single color grayscale tracking using the OpenMV Cam.

#import sensor, image, time, math
#from pyb import LED
#import time
#from pyb import Pin, Timer


#tim_motor = Timer(4, freq=2000) # Frequency in Hz
#tim_servo = Timer(2, freq=300)

##set in1 and in2 to 1 and 0 for now, later need to edit:
#inA = Pin("P0", Pin.OUT_PP)
#inB = Pin("P1", Pin.OUT_PP)
#inA.high()
#inB.low()
#found_finish_line = False

#red_led = LED(1)
#green_led = LED(2)
#blue_led = LED(3)
#ir_led = LED(4)

## Color Tracking Thresholds (Grayscale Min, Grayscale Max)
## The below grayscale threshold is set to only find extremely bright white areas.
#thresholds = (175, 255)

## roi is x, y width, height, weight
#ROIS = [ # [ROI, weight]
        ##(0, 100, 160, 10, 0.7), # You'll need to tweak the weights for your app
        #(0,  50, 160, 15, 0.7), # depending on how your robot is setup.
        #(40, 40, 25, 15, 0.7),
        #(80, 40, 25, 15, 0.7),
        #(35, 55, 25, 15, 0.7), # find finish line on the left
        #(85, 55, 25, 15, 0.7)
        ##(0,  50, 160, 10, 0.7)
       #]
#weight_sum = 0

##pulse_width for motor control:
#motor_pulse_percent = 0
#servo_pulse_percent = 0

#for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
#sensor.reset()
#sensor.set_pixformat(sensor.GRAYSCALE)
#sensor.set_framesize(sensor.QQVGA)
#sensor.skip_frames(time = 2000)
#sensor.set_auto_gain(False) # must be turned off for color tracking
#sensor.set_auto_whitebal(False) # must be turned off for color tracking
#clock = time.clock()

## Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
## returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
## camera resolution. "merge=True" merges all overlapping blobs in the image.
#loop_count = 0
#run_count = 0
#previous_center_x = 0
#pp_center_x = 0
#while(True):
    #loop_count = loop_count + 1
    #clock.tick()
    #img = sensor.snapshot()
    #centroid_sum = 0
    #x_cord = [0,0]
    #y_cord = [0,0]
    #center_x = 0
    #found_line = False
    #finish_blob = 0

    #for r in ROIS:
        #if r == ROIS[0]:
            #blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 100, area_threshold= 100, merge=True) # r[0:4] is roi tuple.
        #else:
            #blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 50, area_threshold= 50, merge=True) # r[0:4] is roi tuple.
        #if blobs:
            ## Find the blob with the most pixels.
            #largest_blob = max(blobs, key=lambda b: b.pixels())

            ### Draw a rect around the blob.
            #img.draw_rectangle(largest_blob.rect(),color=0)
            #img.draw_cross(largest_blob.cx(),largest_blob.cy(),color = 0)
            ##img.draw_cross(60,50,color = 0)
            ##img.draw_cross(100,50,color = 0)
            ##img.draw_cross(55,70,color = 0)
            ##img.draw_cross(105,70,color = 0)
            #img.draw_rectangle(40, 40, 25, 15, color=10)
            #img.draw_rectangle(80, 40, 25, 15, color=10)
            #img.draw_rectangle(35, 55, 25, 15, color=10)
            #img.draw_rectangle(85, 55, 25, 15, color=10)
            #if r == ROIS[0]:
                #if blobs:
                    #center_x = largest_blob.cx()
                    #found_line = True
                #else:
                    #found_line = False
            #if r == ROIS[1]:
                #if blobs:
                    #print('upper left found')
                    #finish_blob = finish_blob + 1
            #if r == ROIS[2]:
                #if blobs:
                    #print('upper right found')
                    #finish_blob = finish_blob + 1
            #if r == ROIS[3]:
                #if blobs:
                    #print('lower left found')
                    #finish_blob = finish_blob + 1
            #if r == ROIS[4]:
                #if blobs:
                    #print('lower right found')
                    #finish_blob = finish_blob + 1
    #center_off = previous_center_x - 80 #p controller
    #d_off = (center_x - pp_center_x) * clock.fps() / 2 #d controller
    ##control algorithm:
    #pidx = center_x + center_off * 0.0 + d_off * 0.000
    #if found_line:
        #if pidx < 30:
            #print('at right')
            #red_led.on()
            #green_led.off()
            #blue_led.off()
            ##motor_pulse_percent = 50 #50 for normal value set to 0 for debug
            ##servo_pulse_percent = 53
        #elif pidx > 120:
            #print('at left')
            #blue_led.on()
            #green_led.off()
            #red_led.off()
            ##motor_pulse_percent = 50
            ##servo_pulse_percent = 37
        #elif pidx <= 95 and pidx >= 55:
            #print('at center')
            #green_led.on()
            #blue_led.off()
            #red_led.off()
            ## motor_pulse_percent = 80
            ## servo_pulse_percent = 45
        #elif pidx < 55 and pidx >= 40:
            #print('near right')
            ## servo_pulse_percent = 49
            ## motor_pulse_percent = 60
            #red_led.on()
            #green_led.on()
            #blue_led.off()
        #elif pidx > 95 and pidx <= 110:
            #print('near left')
            #red_led.off()
            #blue_led.on()
            #green_led.on()

        #motor_pulse_percent = -6.66666666 * 10**-2 * pidx**2 +  10 * pidx - 2.85000 * 10**2
        #servo_pulse_percent =   -5.13194500744* 10**-5 * pidx**3 +   1.1546876266753* 10**-2 * pidx**2 - 9.659644005513 * 10**-1 * pidx +  7.4146544041 * 10**1
        #if motor_pulse_percent <= 40:
            #motor_pulse_percent = 40
        #if servo_pulse_percent >= 55:
            #servo_pulse_percent = 55
        #if servo_pulse_percent <= 35:
            #servo_pulse_percent = 35


    #else:
        #print('nothing detected')
        #green_led.on()
        #red_led.on()
        #blue_led.on()
        #motor_pulse_percent = 50
    ##if finish_blob >= 3:
        ##print('finish line found')
        ##inA.low()
        ##inB.low()
        ##found_finish_line = True
    #if found_finish_line:
        #green_led.on()
        #red_led.on()
        #blue_led.on()
        #time.sleep(100)
        #green_led.off()
        #red_led.off()
        #blue_led.off()
        #time.sleep(100)



    ## for testing purpose, reduce the motor pulse percent so that the car runs slowly:
    #motor_pulse_percent = motor_pulse_percent * 0.75
    #ch1 = tim_motor.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=motor_pulse_percent)
    #ch2 = tim_servo.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=servo_pulse_percent)


    ##calculate the difference of two center points
    #dx = x_cord[0] - x_cord[1]
    #dy = y_cord[0] - y_cord[1]


    #deflection_angle = 0



## left turn is negative and right trun is positive
    #if dy !=0:
        #deflection_angle = -math.atan(dx/dy)

## Convert angle in radians to degrees.
    #deflection_angle = math.degrees(deflection_angle)
#>>>>>>> 633269b58f3366daf1dc04897df1fe9d667171d1
