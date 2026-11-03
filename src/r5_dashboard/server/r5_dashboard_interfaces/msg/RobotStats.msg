# Usage counters and service schedule (spec 4.4, RB-4). Published at 1 Hz on /r5/stats.

# Total time the robot software has been running.
float64 operating_hours
# Total distance driven, from the local odometry.
float64 distance_km

# Service is due every service_interval_km of driving.
float64 service_interval_km
# Distance left before the next service. Negative when overdue.
float64 km_to_service
bool service_due_soon
bool service_overdue

# Last recorded service. has_last_service is false until one has been recorded.
bool has_last_service
builtin_interfaces/Time last_service_time
float64 last_service_distance_km
