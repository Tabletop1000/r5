# Boundary recording by driving (spec FM-4). Published on /r5/recording/status on change and at 1 Hz.

uint8 IDLE=0
# Logging positions while the robot is driven around the edge.
uint8 RECORDING=1
# Stopped. points holds the simplified ring, waiting to be used or discarded.
uint8 FINISHED=2

uint8 state
# While recording: every accepted position. When finished: the simplified, closed ring.
LatLon[] points
# Positions skipped because the fix was worse than required.
uint32 skipped_points
# Distance driven along the track.
float64 length_m
# The fix quality needed for a point to count, e.g. "RTK fixed", and the current one.
string required_fix
string current_fix
# A plain-language note, e.g. why a finished recording is unusable.
string message
builtin_interfaces/Time started
