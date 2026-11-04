# A field (spec 4.1 field model). Stored on the robot as GeoJSON in WGS84 (spec D5, FM-8).

# Set by the robot when the field is first saved. Empty for a new field.
string id
string name
LatLon[] boundary
# Known obstacles and no-go zones, each inside the boundary.
GeoPolygon[] keep_out
# Width of the headland (turnrow) strip inside the boundary, where the robot turns.
float64 headland_width_m
# "drawn", "recorded" or "imported".
string source
string notes

# Filled in by the robot on save.
float64 area_m2
builtin_interfaces/Time created
builtin_interfaces/Time modified
