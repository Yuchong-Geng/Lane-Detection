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
inA.low()
inB.high()

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (175, 255)


ROIS = [ # [ROI, weight]
        #(0, 100, 160, 10, 0.7), # You'll need to tweak the weights for your app
        (0,  50, 160, 10, 0.7), # depending on how your robot is setup.
        (0,   0, 160, 10, 0.7),
        (50, 60, 15, 120, 0.7), # find finish line on the left
        (110, 60, 15, 120, 0.7),
        (0,  50, 160, 10, 0.7)
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
i = 0
loop_count = 0
run_count = 0
previous_center_x = 0
pp_center_x = 0
while(True):
    clock.tick()
    img = sensor.snapshot()
    centroid_sum = 0
    #i = i + 1
    #if i == 40:
        #red_led.toggle()
        #i = 0;
    #for blob in img.find_blobs([thresholds], pixels_threshold= 100, area_threshold= 100, merge=True):
        #These values depend on the blob not being circular - otherwise they will be shaky.
    #    if blob.elongation() > 0.5:
    #       img.draw_edges(blob.min_corners(), color=0)
    #       img.draw_line(blob.major_axis_line(), color=0)
    #       img.draw_line(blob.minor_axis_line(), color=0)
        #These values are stable all the time.
    #img.draw_rectangle(blob.rect(), color=0)
    #img.draw_cross(blob.cx(), blob.cy(), color=0)
         #Note - the blob rotation is unique to 0-180 only.
    #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=40, color=127)
    x_cord = [0,0]
    y_cord = [0,0]
    # found_finish_line = [False, False]
    ##pulse_width for motor control:
    #motor_pulse_percent = 0
    #servo_pulse_percent = 0
    center_x = 0
    found_line = False



    for r in ROIS:
        blobs = img.find_blobs([thresholds], roi=r[0:4],pixels_threshold= 100, area_threshold= 100, merge=True) # r[0:4] is roi tuple.
        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect(),color=0)
            img.draw_cross(largest_blob.cx(),largest_blob.cy(),color = 0)

            #
            if r == ROIS[0]:
                x_cord[0] = largest_blob.cx()
                y_cord[0] = largest_blob.cy()
            if r == ROIS[1]:
                x_cord[1] = largest_blob.cx()
                y_cord[1] = largest_blob.cy()
            # #find if finished line found:
            # if r == ROIS[2]:
            #     if blobs:
            #         found_finish_line[0] = True
            # if r == ROIS[3]:
            #     if blobs:
            #         found_finish_line[1] = True
            #find the car's position relative to the center line:
            if r == ROIS[4]:
                if blobs:
                    found_line = True
                    center_x = largest_blob.cx()
                else:
                     found_line = False
    center_off = previous_center_x - 80 #p controller
    d_off = (center_x - pp_center_x) * clock.fps() / 2 #d controller
    print('center_x: ' + str(center_x))
    #print('d_off: ' + str(d_off))
    if loop_count != 0:
        pp_center_x = previous_x
    #load previous_Center_x for next term
    previous_center_x = center_x
    #control algorithm:
    pidx = center_x + center_off * 0.1 + d_off * 0
    #print("center_off " + str(center_off))
    #print("pidx " + str(pidx))
    if found_line:
        if pidx <= 95 and pidx >= 45:
            print('at center')
            green_led.on()
            blue_led.off()
            red_led.off()
            motor_pulse_percent = 80
            servo_pulse_percent = 45
        elif pidx < 45 and pidx >= 30:
            print('near right')
            servo_pulse_percent = 50
            motor_pulse_percent = 60
            red_led.on()
            green_led.on()
            blue_led.off()
        elif pidx < 30:
            print('at right')
            red_led.on()
            green_led.off()
            blue_led.off()
            motor_pulse_percent = 50 #50 for normal value set to 0 for debug
            servo_pulse_percent = 55
        elif pidx > 95 and pidx <= 110:
            print('near left')
            red_led.off()
            blue_led.on()
            green_led.on()
            motor_pulse_percent = 60
            servo_pulse_percent = 40
        elif pidx > 110:
            print('at left')
            blue_led.on()
            green_led.off()
            red_led.off()
            motor_pulse_percent = 50
            servo_pulse_percent = 35
    else:
        print('nothing detected')
        green_led.on()
        red_led.on()
        blue_led.on()
        motor_pulse_percent = 50

    #if found_finish_line == [True, True]:
        #green_led.on()
        #blue_led.on()
        #red_led.on()
        #print('finish line detected')
        #motor_pulse_percent = 0


    # for testing purpose, reduce the motor pulse percent so that the car runs slowly:
    motor_pulse_percent = motor_pulse_percent * 0.5
    ch1 = tim_motor.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=motor_pulse_percent)
    ch2 = tim_servo.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=servo_pulse_percent)


    #calculate the difference of two center points
    dx = x_cord[0] - x_cord[1]
    dy = y_cord[0] - y_cord[1]


    deflection_angle = 0



# left turn is negative and right trun is positive
    if dy !=0:
        deflection_angle = -math.atan(dx/dy)

# Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)

# Now you have an angle telling you how much to turn the robot by which
# incorporates the part of the line nearest to the robot and parts of
# the line farther away from the robot for a better prediction.
    #print("Turn Angle: %f" % deflection_angle)
    #print(center_x)
    #print(clock.fps())
    #print(found_finish_line)
    # Note: Your OpenMV Cam runs about half as fast while
# connected to your computer. The FPS should increase once disconnected.
