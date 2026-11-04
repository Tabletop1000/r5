# A planned area spray job (spec AS-3, AS-4). Published latched on /r5/job/plan for the active job.
uint32 plan_id
string field_id
string field_name
SpraySettings settings
PathSection[] sections
# The row direction used, degrees clockwise from north.
float64 row_angle_deg
uint32 swath_count
float64 area_sprayed_m2
float64 spray_length_m
float64 path_length_m
# Estimated time including trips home to refill.
float64 duration_s
float64 liquid_needed_l
uint32 refills
# Pump flow at the planned speed, L/min (spec AS-1).
float64 flow_l_min
string[] warnings
