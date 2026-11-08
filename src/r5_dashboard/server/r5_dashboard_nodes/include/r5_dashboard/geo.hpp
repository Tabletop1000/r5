#ifndef R5_DASHBOARD__GEO_HPP_
#define R5_DASHBOARD__GEO_HPP_

#include <vector>

// Small geometry helpers for fields (spec FM-4, FM-6), without ROS.
//
// Fields are a few hundred metres across, so positions are projected onto a flat local grid in
// metres (equirectangular, centred on the shape). The error from this is far below GPS accuracy at
// that size. geo.py (still used by job_manager) and web/src/fields/geo.ts follow the same rules.
namespace r5_dashboard::geo
{

constexpr double kEarthRadiusM = 6371008.8;

struct LatLon
{
  double lat = 0.0;  // degrees
  double lon = 0.0;
  bool operator==(const LatLon & o) const {return lat == o.lat && lon == o.lon;}
};

struct XY
{
  double x = 0.0;  // metres east
  double y = 0.0;  // metres north
  bool operator==(const XY & o) const {return x == o.x && y == o.y;}
};

using Ring = std::vector<XY>;  // Closed; the first point isn't repeated.

double distance(const XY & a, const XY & b);

// Flat x/y in metres around an origin. Accurate to well under a centimetre over a few km.
class LocalProjection
{
public:
  explicit LocalProjection(const LatLon & origin);
  // Centred on the average of the points, which mustn't be empty.
  static LocalProjection around(const std::vector<LatLon> & points);

  XY to_xy(const LatLon & p) const;
  LatLon to_latlon(const XY & p) const;
  std::vector<XY> to_xy(const std::vector<LatLon> & points) const;

private:
  LatLon origin_;
  double kx_;
  double ky_;
};

// Unsigned area of a closed ring.
double ring_area_m2(const Ring & ring);
double path_length_m(const std::vector<XY> & points);
bool segments_intersect(const XY & p1, const XY & p2, const XY & q1, const XY & q2);
// True if any two non-neighbouring edges of the closed ring touch or cross.
bool ring_self_intersects(const Ring & ring);
bool rings_cross(const Ring & a, const Ring & b);
bool point_in_ring(const XY & p, const Ring & ring);
bool ring_inside(const Ring & inner, const Ring & outer);
// Douglas-Peucker: drops points while staying within tolerance of the original line.
std::vector<XY> simplify(const std::vector<XY> & points, double tolerance);
// Turns a driven track into a simplified ring: joins its end to its start, then simplifies.
// Repeated end points are dropped.
Ring close_track(const std::vector<XY> & track, double tolerance);

}  // namespace r5_dashboard::geo

#endif  // R5_DASHBOARD__GEO_HPP_
