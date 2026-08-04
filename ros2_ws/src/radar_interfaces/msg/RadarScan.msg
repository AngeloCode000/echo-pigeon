# One radar frame: all detections sharing a single timestamp.
# An empty detections array is a valid frame with no detections
# (the tracker's missed-detection signal).

std_msgs/Header header

RadarDetection[] detections
