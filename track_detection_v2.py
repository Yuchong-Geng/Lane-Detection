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
        (0,  80, 160, 15, 0.7) # depending on how your robot is setup
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
speed_index = 0.75
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

    for r in ROIS:
        if r == ROIS[0]:
            blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 80, area_threshold= 80, merge=True) # r[0:4] is roi tuple.
            if blobs:
                # Find the blob with the most pixels.
                largest_blob = max(blobs, key=lambda b: b.pixels())
                center_x = largest_blob.cx()
                found_line = True
                ## Draw a rect around the blob.
                #img.draw_rectangle(largest_blob.rect(),color=0)
                #img.draw_cross(largest_blob.cx(),largest_blob.cy(),color = 0)

    print('center _x: ' + str(center_x))
    pidx = center_x
    if found_line:
        if pidx < 63.48 and pidx > 0:
            print('at right')
            red_led.on()
            green_led.off()
            blue_led.off()
            if state == 'not braking':
                start_brake_index = loop_count
                end_brake_index = start_brake_index + 10
                state = 'braking'
        elif pidx > 96.52:
            print('at left')
            blue_led.on()
            green_led.off()
            red_led.off()
            if state == 'not braking':
                start_brake_index = loop_count
                end_brake_index = start_brake_index + 10
                state = 'braking'
        elif pidx <= 88.6 and pidx >= 71.74:
            print('at center')
            green_led.on()
            blue_led.off()
            red_led.off()
            state = 'not braking'
        elif pidx < 71.74 and pidx >= 63.48:
            print('near right')
            red_led.on()
            green_led.on()
            blue_led.off()
            state = 'not braking'
        elif pidx > 88.6 and pidx <= 96.52:
            print('near left')
            red_led.off()
            blue_led.on()
            green_led.on()
            state = 'not braking'

        motor_pulse_percent = -7.22222222 * 10**-2 * pidx**2 +  1.08333333 * 10 * pidx - 3.11249999 * 10**2
        servo_pulse_percent =  -1.658218* 10**-4 * pidx**3 +    4.241605* 10**-2 * pidx**2 - 3.641799* 10**0 * pidx +  1.49771063* 10**2
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



    # for testing purpose, reduce the motor pulse percent so that the car runs slowly:
    motor_pulse_percent = motor_pulse_percent * speed_index
    #if found_finish_line:
        #motor_pulse_percent = 0
    ch1 = tim_motor.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=motor_pulse_percent)
    ch2 = tim_servo.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=servo_pulse_percent)

    print(clock.fps())
