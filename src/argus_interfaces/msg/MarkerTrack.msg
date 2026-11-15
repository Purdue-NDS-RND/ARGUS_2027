# One localisation marker as the marker tracker (LOC-7) currently believes it to be.

# Track states.
uint8 TENTATIVE=0   # seen, not yet confirmed
uint8 CONFIRMED=1   # seen often enough to be listed
uint8 COASTING=2    # confirmed, but not seen lately
uint8 LOST=3        # not seen for too long; about to be dropped

# Stamp of the latest update; frame_id is the local ENU frame.
std_msgs/Header header
# Stable for the life of the track.
uint32 track_id
# One of the constants above.
uint8 state
# Position in the local ENU frame, metres.
geometry_msgs/Point position_enu
# 2 x 2 covariance of east and north, row-major, metres squared.
float64[4] covariance_en
# Log probability of each digit 0 to 9, summed over sightings.
float32[10] digit_log_prob
# The digit read, or -1 when unknown.
int8 digit
# How many detections have been matched to this track.
uint32 sightings
# True when the track is outside the survey area and must stay off the judges' list.
bool outside
