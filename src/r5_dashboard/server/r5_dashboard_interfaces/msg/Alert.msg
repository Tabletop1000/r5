# One active problem, shown on the Robot page and counted in the status strip (spec RB-2).

uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2

# Stable identifier, e.g. the diagnostic name "/Sensors/IMU". Used to acknowledge it.
string id
uint8 severity
# Where it comes from, in plain words, e.g. "IMU".
string source
string message
# When this alert started (robot wall clock).
builtin_interfaces/Time since
bool acknowledged
