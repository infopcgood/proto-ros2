float64 time
int32 id

float32[2] resolution

float32 right_eye_open
float32 left_eye_open

float32[4] right_gaze
float32[4] left_gaze

bool got_3d_points
float32 fit_3d_error

float32[3] rotation
float32[3] translation
float32[4] raw_quaternion
float32[3] raw_euler

float32[68] confidence
geometry_msgs/Point[68] points
geometry_msgs/Point[70] points3d

float32 eye_left
float32 eye_right

float32 eyebrow_steepness_left
float32 eyebrow_updown_left
float32 eyebrow_quirk_left

float32 eyebrow_steepness_right
float32 eyebrow_updown_right
float32 eyebrow_quirk_right

float32 mouth_corner_updown_left
float32 mouth_corner_inout_left

float32 mouth_corner_updown_right
float32 mouth_corner_inout_right

float32 mouth_open
float32 mouth_wide
