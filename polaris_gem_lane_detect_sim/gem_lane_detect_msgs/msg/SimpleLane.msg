## This expresses the estimated curve representing the lane center curve w.r.t
# the chosen reference frame on the ego vehicle.
# To uniquely define the lane and avoid confusions in positive and negative
# values, we first define the ego vehicle w.r.t the lane center curve and apply
# the coordinate transformation at the end.
# First, we assume the lane center line is a (half) circle with the center
# point C and the radius r>0, and the reference point of ego car is at point P.
# We denote the intersection of the extended line C-P with the circle as the
# origin O. Given the origin O, we define O->C direction as the y-axis and
# the respective tangential vector as the x-axis using right hand rule.
# we can then derive the heading angle of the vehicle `yaw` w.r.t x-axis.
# Further, P should lie on the y-axis by definition and hence O->P vector is
# perpendicular to the tangent line. We therefore can use O->P vector as the
# lateral deviation `d`. We then transform to the ego vehicle coordinates and
# derive:
#
#   yaw_err = -yaw  (rad in [-pi, pi])
#   offset = -d (meters)
#   curvature = 1/r, when curvature==0, the curve collapse to a straight line.
#
# We can further assume |d| << r, that is, check |offset|*curvature << 1, so we
# can exclude the case that there are two tangent lines to choose when the
# vehicle is at the center of the circle.
#
# Note that for pure pursuit controller, the reference point is defined on the
# center of the rear axle of the ego vehicle. For the Stanley controller, the
# reference point is however defined on the center of the front axle.
# We recommend using SimpleLaneStamped message to also include the reference
# frame.

float32 yaw_err      # estimated heading error (rad in [-pi, pi]).
float32 offset       # estimated lateral offset, i.e., cross track error (m).
float32 curvature    # estimated curvature (m^-1). The value is always >=0.
