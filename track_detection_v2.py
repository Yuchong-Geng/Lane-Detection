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
        (50, 40, 15, 30, 0.7),
        (95, 40, 15, 30, 0.7)
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
speed_index = 0.70
start_brake_index = 0
end_brake_index = 0
state = 'not braking'
while(True):
    loop_count = loop_count + 1
    clock.tick()
    img = sensor.snapshot()
    centroid_sum = 0
    x_cord = [0,0]
    y_cord = [0,0]
    center_x = 0
    found_line = False
    far_finish_found = False
    finish_blob = 0
    far_finish = 0

    for r in ROIS:
        if r == ROIS[0]:
            blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 100, area_threshold= 100, merge=True) # r[0:4] is roi tuple.
        #elif r == ROIS[5] or r == ROIS[6]:
            #blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 10, area_threshold= 10, merge=True) # r[0:4] is roi tuple.
        else:
            blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 130, area_threshold= 130, merge=True) # r[0:4] is roi tuple.
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
            img.draw_rectangle(50, 40, 15, 30, color=10)
            img.draw_rectangle(95, 40, 15, 30, color=10)
            #img.draw_rectangle(35, 55, 25, 15, color=10)
            #img.draw_rectangle(85, 55, 25, 15, color=10)
            #img.draw_rectangle(80, 20, 10, 15, color=10)
            #img.draw_rectangle(60, 20, 10, 15, color=10)
            if r == ROIS[0]:
                if blobs:
                    center_x = largest_blob.cx()
                    found_line = True
                else:
                    found_line = False
            if r == ROIS[1]:
                if blobs:
                    print('left found')
                    finish_blob = finish_blob + 1
            if r == ROIS[2]:
                if blobs:
                    print('right found')
                    finish_blob = finish_blob + 1
    center_off = previous_center_x - 80 #p controller
    d_off = (center_x - pp_center_x) * clock.fps() / 2 #d controller
    #control algorithm:
    pidx = center_x + center_off * 0.0 + d_off * 0.000
    if found_line:
        if pidx < 33.69:
            print('at right')
            red_led.on()
            green_led.off()
            blue_led.off()
            if state == 'not braking':
                start_brake_index = loop_count
                end_brake_index = start_brake_index + 7
                state = 'braking'
        elif pidx > 116.31:
            print('at left')
            blue_led.on()
            green_led.off()
            red_led.off()
            if state == 'not braking':
                start_brake_index = loop_count
                end_brake_index = start_brake_index + 7
                state = 'braking'
        elif pidx <= 95 and pidx >= 55:
            print('at center')
            green_led.on()
            blue_led.off()
            red_led.off()
            state = 'not braking'
        elif pidx < 55 and pidx >= 40:
            print('near right')
            red_led.on()
            green_led.on()
            blue_led.off()
            state = 'not braking'
        elif pidx > 95 and pidx <= 110:
            print('near left')
            red_led.off()
            blue_led.on()
            green_led.on()
            state = 'not braking'

        motor_pulse_percent = -7.22222222 * 10**-2 * pidx**2 +  1.08333333 * 10 * pidx - 3.11249999 * 10**2
        servo_pulse_percent =   -8.4379648* 10**-5 * pidx**3 +   1.898542090* 10**-2 * pidx**2 - 1.473569049* 10**0 * pidx +    8.432235029* 10**1
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
    #braking to make sure car does not understeer when approching turns:
    if state == 'braking' and loop_count <= end_brake_index:
        inA.low()
        inB.low()
    else:
        inA.high()
        inB.low()
    #finish line detection method:
    #if finish_blob == 2:
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


    ##calculate the difference of two center points
    #dx = x_cord[0] - x_cord[1]
    #dy = y_cord[0] - y_cord[1]


    #deflection_angle = 0
