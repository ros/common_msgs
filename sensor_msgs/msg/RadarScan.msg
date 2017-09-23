# Single horizontal scan from a scanning radar (electronic or mechanical)
#
# This message is assumed to be generated from a horizontally- and/or vertically-scanning radar and
# covers all measurements taken within a single scan.

Header header                             # Timestamp in the header is the time of the beginning of the scan.

uint8 CLOCKWISE=0
uint8 COUNTER_CLOCKWISE=1

uint8 horizontal_scan_direction           # This indicates whether the scan proceeded clockwise or counter-clockwise
                                          # across the horizontal field of view about the z axis of the radar.
                                          # This influences the order in which the measurements are stored in the arrays
                                          # unless the radar has no horizontal discrimination.

uint8 TOP_TO_BOTTOM=0
uint8 BOTTOM_TO_TOP=1

uint8 vertical_scan_direction             # This indicates whether the scan proceeded down from the top of the radar
                                          # (TOP_TO_BOTTOM) or up from the bottom of the radar (BOTTOM_TO_TOP) about
                                          # the y axis of the radar. This influences the order in which the measurements
                                          # are stored in the arrays unless the radar has no vertical discrimination.

float32 horizontal_field_of_view          # The size of the arc of the entire horizontal field of view of the radar [rad].
                                          # The measurements described in this message fall entirely within
                                          # -horizontal_field_of_view/2 and horizontal_field_of_view/2.
                                          # The x plane intersects the radar vertically along it's midpoint (boresight).

float32 vertical_field_of_view            # The size of the arc of the entire vertical field of view of the radar [rad].
                                          # The measurements described in this message fall entirely within
                                          # -vertical_field_of_view/2 and vertical_field_of_view/2.
                                          # The z plane intersects the radar horizontally along it's midpoint (boresight).

# In the arrays below, the ordering is horizontal first, then vertical. However, the ordering of the horizontal
# detections is dependent upon the horizontal scan direction above and the same for the vertical.

geometry_msgs/Point[] locations           # The location of each detection relative to the center of the radar.
                                          #
                                          # Since a radar will generally report the location of a detection in a polar
                                          # coordinate system, the raw values must be converted to Euclidean space.

geometry_msgs/Vector3[] linear_velocities # The linear velocities of each detection relative to the radar.
                                          #
                                          # Since a radar will generally report the velocities of a detection in a polar
                                          # coordinate system, the raw values must be converted to Euclidean space.
                                          # Because the velocity measured in a single scan can only measure the change in
                                          # radial distance along the polar and azimuthal angles (and not the change in
                                          # polar or azimuthal angles at the same radial distance - which would require
                                          # multiple scans), and because the standard ROS messages do not currently support
                                          # representations of coordinates in a polar coordinate frame, the representation
                                          # of velocities found here is just a rough approximation of the real velocities
                                          # in Euclidean space based on the measured, radial velocities.

float32[] amplitudes                      # The amplitude of each detection [dB].
