# How long one stage of the vision chain took for one frame (latency probe, PLT-05).

# Stamp of the frame the timing belongs to.
std_msgs/Header header
# Name of the stage, for example "capture", "detect" or "estimate".
string stage
# Time spent in the stage, milliseconds.
float32 latency_ms
