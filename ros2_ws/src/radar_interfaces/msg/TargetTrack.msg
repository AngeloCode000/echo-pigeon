# A persistent target track produced by target_tracker.
# State layout matches project_plan.md Phase 3 "Track state".

std_msgs/Header header

uint32 track_id

float64 position_x
float64 position_y
float64 position_z
float64 velocity_x
float64 velocity_y
float64 velocity_z

# Row-major 6x6 state covariance.
float64[36] covariance

builtin_interfaces/Duration track_age
uint32 detection_count
uint32 miss_count
float64 confidence
