#include "r5_dashboard/geo.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace r5_dashboard::geo
{

namespace
{

constexpr double kDegree = M_PI / 180.0;

double orient(const XY & a, const XY & b, const XY & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool on_segment(const XY & a, const XY & b, const XY & p)
{
  return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

double distance_to_segment(const XY & p, const XY & a, const XY & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  if (dx == 0 && dy == 0) {
    return distance(p, a);
  }
  const double t =
    std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy)));
  return distance(p, {a.x + t * dx, a.y + t * dy});
}

}  // namespace

double distance(const XY & a, const XY & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

LocalProjection::LocalProjection(const LatLon & origin)
: origin_(origin),
  kx_(kDegree * kEarthRadiusM * std::cos(origin.lat * kDegree)),
  ky_(kDegree * kEarthRadiusM)
{
}

LocalProjection LocalProjection::around(const std::vector<LatLon> & points)
{
  double lat = 0.0;
  double lon = 0.0;
  for (const auto & p : points) {
    lat += p.lat;
    lon += p.lon;
  }
  const double n = static_cast<double>(points.size());
  return LocalProjection({lat / n, lon / n});
}

XY LocalProjection::to_xy(const LatLon & p) const
{
  return {(p.lon - origin_.lon) * kx_, (p.lat - origin_.lat) * ky_};
}

LatLon LocalProjection::to_latlon(const XY & p) const
{
  return {origin_.lat + p.y / ky_, origin_.lon + p.x / kx_};
}

std::vector<XY> LocalProjection::to_xy(const std::vector<LatLon> & points) const
{
  std::vector<XY> out;
  out.reserve(points.size());
  for (const auto & p : points) {
    out.push_back(to_xy(p));
  }
  return out;
}

double ring_area_m2(const Ring & ring)
{
  const std::size_t n = ring.size();
  double s = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const XY & a = ring[i];
    const XY & b = ring[(i + 1) % n];
    s += a.x * b.y - b.x * a.y;
  }
  return std::abs(s) / 2.0;
}

double path_length_m(const std::vector<XY> & points)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    total += distance(points[i], points[i + 1]);
  }
  return total;
}

bool segments_intersect(const XY & p1, const XY & p2, const XY & q1, const XY & q2)
{
  const double d1 = orient(q1, q2, p1);
  const double d2 = orient(q1, q2, p2);
  const double d3 = orient(p1, p2, q1);
  const double d4 = orient(p1, p2, q2);
  if (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 &&
    d4 != 0)
  {
    return true;
  }
  return (d1 == 0 && on_segment(q1, q2, p1)) || (d2 == 0 && on_segment(q1, q2, p2)) ||
         (d3 == 0 && on_segment(p1, p2, q1)) || (d4 == 0 && on_segment(p1, p2, q2));
}

bool ring_self_intersects(const Ring & ring)
{
  const std::size_t n = ring.size();
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      if (j == i + 1 || (i == 0 && j == n - 1)) {
        continue;  // Neighbours share a corner.
      }
      if (segments_intersect(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n])) {
        return true;
      }
    }
  }
  return false;
}

bool rings_cross(const Ring & a, const Ring & b)
{
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      if (segments_intersect(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()])) {
        return true;
      }
    }
  }
  return false;
}

bool point_in_ring(const XY & p, const Ring & ring)
{
  bool inside = false;
  const std::size_t n = ring.size();
  for (std::size_t i = 0; i < n; ++i) {
    const XY & a = ring[i];
    const XY & b = ring[(i + 1) % n];
    if ((a.y > p.y) != (b.y > p.y)) {
      const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool ring_inside(const Ring & inner, const Ring & outer)
{
  return std::all_of(inner.begin(), inner.end(), [&outer](const XY & p) {
             return point_in_ring(p, outer);
           }) && !rings_cross(inner, outer);
}

std::vector<XY> simplify(const std::vector<XY> & points, double tolerance)
{
  if (points.size() < 3) {
    return points;
  }
  std::vector<bool> keep(points.size(), false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<std::size_t, std::size_t>> stack = {{0, points.size() - 1}};
  while (!stack.empty()) {
    const auto [start, end] = stack.back();
    stack.pop_back();
    double worst = 0.0;
    std::size_t index = 0;
    bool found = false;
    for (std::size_t i = start + 1; i < end; ++i) {
      const double d = distance_to_segment(points[i], points[start], points[end]);
      if (d > worst) {
        worst = d;
        index = i;
        found = true;
      }
    }
    if (found && worst > tolerance) {
      keep[index] = true;
      stack.emplace_back(start, index);
      stack.emplace_back(index, end);
    }
  }
  std::vector<XY> out;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) {
      out.push_back(points[i]);
    }
  }
  return out;
}

Ring close_track(const std::vector<XY> & track, double tolerance)
{
  std::vector<XY> pts = track;
  if (pts.size() >= 2 && distance(pts.front(), pts.back()) < 1e-6) {
    pts.pop_back();
  }
  if (pts.size() < 3) {
    return pts;
  }
  pts.push_back(pts.front());
  Ring ring = simplify(pts, tolerance);
  ring.pop_back();
  return ring;
}

}  // namespace r5_dashboard::geo
