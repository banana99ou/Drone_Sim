# A downward optical-flow sensor, modelled on a PMW3901-class module.
#
# What such a sensor actually reports is not a velocity. It reports how far the
# IMAGE moved, as an angle, integrated over a short interval -- and that angle
# depends on height, which the sensor does not know. Turning it into a velocity
# needs a range measurement, which is why these modules are always paired with
# a rangefinder and why ground_distance_m travels with the reading.
#
#     v_body_x ~= (integrated_y - integrated_ygyro) / integration_time_s
#                 * ground_distance_m
#     v_body_y ~= -(integrated_x - integrated_xgyro) / integration_time_s
#                 * ground_distance_m
#
# The gyro terms are there because the sensor cannot tell translation from
# rotation: tilting the airframe sweeps the image exactly as sliding it does.
# Both the raw flow and the gyro integral over the same interval are published
# so a consumer can subtract one from the other, which is what the real
# hardware expects you to do. Publishing a pre-compensated value would hide
# the failure mode -- a gyro/flow timing mismatch shows up as phantom velocity
# during rotation, and you can only see that if you get both numbers.
#
# Sign convention, stated because every vendor picks a different one:
#   integrated_x  flow about the body +x axis, so motion along body +y is
#                 NEGATIVE
#   integrated_y  flow about the body +y axis, so motion along body +x is
#                 POSITIVE
#
# quality is 0 when the reading must not be used at all: too low, too high, or
# tilted far enough that the ground has left the field of view. It is not a
# confidence to be weighted -- 0 means "no measurement", and a consumer that
# treats it as "a poor measurement" will fly into things.

std_msgs/Header header

float64 integration_time_s     # interval this reading covers
float64 integrated_x           # rad, flow about body +x over the interval
float64 integrated_y           # rad, flow about body +y
float64 integrated_xgyro       # rad, gyro integral over the SAME interval
float64 integrated_ygyro       # rad
float64 ground_distance_m      # the range the flow must be scaled by; NaN if
                               # the rangefinder had no valid return
uint8 quality                  # 0 = unusable, 255 = best
