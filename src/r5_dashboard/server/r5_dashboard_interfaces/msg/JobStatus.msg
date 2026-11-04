# The current spray job. Published latched on /r5/job/status on change and at 1 Hz (spec AS-6, AS-7).

uint8 IDLE=0
uint8 GOING_TO_START=1
uint8 SPRAYING=2
uint8 TURNING=3
uint8 PAUSED=4
uint8 RETURNING_HOME=5
uint8 WAITING_FOR_REFILL=6
uint8 RETURNING_TO_WORK=7
uint8 FINISHED=8
uint8 FAILED=9
uint8 STOPPED=10

uint8 state
# Plain-language summary for the status strip, e.g. "Spraying East field — 42%" (spec NAV-3).
string current_action
string message
string field_id
string field_name
uint32 plan_id
# Index into the plan's sections of the one being driven.
uint32 section_index
uint32 swaths_done
uint32 swath_count
# 0–1, by sprayed length.
float64 progress
float64 area_done_m2
float64 liquid_used_l
# Estimated, from flow and spraying time (no tank sensor yet).
float64 tank_remaining_l
float64 tank_capacity_l
float64 remaining_s
bool sprayer_on
builtin_interfaces/Time started
