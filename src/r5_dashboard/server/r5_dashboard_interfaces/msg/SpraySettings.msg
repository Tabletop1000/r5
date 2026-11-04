# Area spray job settings (spec 4.2 inputs).
string field_id
# Litres per square metre (spec Q5).
float64 application_rate_l_per_m2
float64 boom_width_m
float64 speed_kmh
float64 tank_capacity_l
# true: let the planner pick the row direction with the fewest turns.
bool auto_angle
# Row direction when auto_angle is false: degrees clockwise from north.
float64 row_angle_deg
