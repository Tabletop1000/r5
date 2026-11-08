#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "r5_dashboard/fields.hpp"
#include "r5_dashboard/geo.hpp"
#include "temp_dir.hpp"

using namespace r5_dashboard::fields;  // NOLINT
using namespace r5_dashboard::geo;  // NOLINT

namespace
{

const LatLon kOrigin{-33.86, 151.21};
const LocalProjection kProj(kOrigin);

// Metres east/north from kOrigin to lat/lon.
std::vector<LatLon> ll(const std::vector<XY> & points)
{
  std::vector<LatLon> out;
  for (const auto & p : points) {
    out.push_back(kProj.to_latlon(p));
  }
  return out;
}

const std::vector<LatLon> kSquare100 = ll({{0, 0}, {100, 0}, {100, 100}, {0, 100}});  // 1 ha

Field make_field()
{
  Field f;
  f.name = "East field";
  f.boundary = kSquare100;
  f.headland_width_m = 2.0;
  return f;
}

std::string invalid_reason(const Field & f, const std::vector<std::string> & others = {})
{
  try {
    validate(f, others);
  } catch (const FieldInvalid & e) {
    return e.what();
  }
  return "";
}

bool sorted_equal(Ring a, Ring b)
{
  const auto less = [](const XY & p, const XY & q) {
      return p.x < q.x || (p.x == q.x && p.y < q.y);
    };
  std::sort(a.begin(), a.end(), less);
  std::sort(b.begin(), b.end(), less);
  return a == b;
}

}  // namespace

// ---- geo ----

TEST(Geo, ProjectionRoundTripAndScale)
{
  const auto p = kProj.to_latlon({100.0, 50.0});
  const auto xy = kProj.to_xy(p);
  EXPECT_NEAR(xy.x, 100.0, 1e-6);
  EXPECT_NEAR(xy.y, 50.0, 1e-6);
  // One degree of latitude is about 111 km.
  EXPECT_NEAR(kProj.to_xy({kOrigin.lat + 1, kOrigin.lon}).y, 111195, 111);
}

TEST(Geo, AreaSelfIntersectionAndContainment)
{
  const Ring square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_EQ(ring_area_m2(square), 100);
  EXPECT_FALSE(ring_self_intersects(square));
  EXPECT_TRUE(ring_self_intersects({{0, 0}, {10, 10}, {10, 0}, {0, 10}}));  // bow tie
  EXPECT_TRUE(point_in_ring({5, 5}, square));
  EXPECT_FALSE(point_in_ring({15, 5}, square));
  EXPECT_TRUE(ring_inside({{2, 2}, {4, 2}, {4, 4}}, square));
  EXPECT_FALSE(ring_inside({{8, 8}, {12, 8}, {12, 12}}, square));
}

TEST(Geo, SimplifyKeepsShapeWithinTolerance)
{
  std::vector<XY> line;
  for (int x = 0; x <= 100; ++x) {
    line.push_back({x / 10.0, 0.02 * std::sin(x)});  // 2 cm wiggle along 10 m
  }
  EXPECT_EQ(simplify(line, 0.1), (std::vector<XY>{line.front(), line.back()}));
  const std::vector<XY> corner = {{0, 0}, {5, 0}, {10, 0}, {10, 5}, {10, 10}};
  EXPECT_EQ(simplify(corner, 0.1), (std::vector<XY>{{0, 0}, {10, 0}, {10, 10}}));
}

TEST(Geo, CloseTrackDropsRepeatedEndAndCollinearPoints)
{
  const std::vector<XY> track = {{0, 0}, {5, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 5}, {0, 0}};
  EXPECT_TRUE(sorted_equal(close_track(track, 0.1), {{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
}

// ---- validation ----

TEST(Validate, ValidFieldPasses)
{
  Field f = make_field();
  f.keep_out = {ll({{40, 40}, {60, 40}, {60, 60}})};
  EXPECT_EQ(invalid_reason(f), "");
}

TEST(Validate, InvalidFields)
{
  struct Case
  {
    Field field;
    std::string reason;
  };
  std::vector<Case> cases;
  auto add = [&cases](auto change, const std::string & reason) {
      Field f = make_field();
      change(f);
      cases.push_back({f, reason});
    };
  add([](Field & f) {f.name = "  ";}, "Give the field a name.");
  add([](Field & f) {f.name = std::string(61, 'x');},
    "The name is too long (at most 60 characters).");
  add([](Field & f) {f.boundary.resize(2);}, "The boundary needs at least 3 points.");
  add([](Field & f) {f.boundary = ll({{0, 0}, {100, 100}, {100, 0}, {0, 100}});},
    "The boundary crosses itself.");
  add([](Field & f) {f.keep_out = {ll({{90, 90}, {120, 90}, {120, 120}})};},
    "Keep-out zone 1 must lie inside the boundary.");
  add([](Field & f) {
      f.keep_out = {ll({{10, 10}, {30, 10}, {30, 30}}), ll({{20, 20}, {40, 20}, {40, 40}})};
    }, "Keep-out zones 1 and 2 overlap. Merge them into one.");
  add([](Field & f) {f.headland_width_m = 25.0;},
    "The headland width must be between 0 and 20 m.");
  add([](Field & f) {f.headland_width_m = std::numeric_limits<double>::quiet_NaN();},
    "The headland width must be between 0 and 20 m.");
  add([](Field & f) {f.source = "magic";}, "Unknown field source “magic”.");
  add([](Field & f) {f.boundary = {{95.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}};},
    "A point is outside the valid range of latitude and longitude.");
  for (const auto & c : cases) {
    EXPECT_EQ(invalid_reason(c.field), c.reason);
  }
}

TEST(Validate, DuplicateNameIsCaseInsensitive)
{
  Field f = make_field();
  f.name = "east FIELD";
  EXPECT_EQ(invalid_reason(f, {"East field"}), "There is already a field called “east FIELD”.");
}

// ---- storage ----

TEST_F(TempDir, FieldStoreSaveListUpdateDelete)
{
  std::vector<double> times = {1000.0, 1000.0, 2000.0, 2000.0};
  std::size_t next = 0;
  FieldStore store(dir_, [&] {return times.at(next++);});
  Field f = make_field();
  f.keep_out = {ll({{40, 40}, {60, 40}, {60, 60}})};
  Field saved = store.save(f);
  EXPECT_FALSE(saved.id.empty());
  EXPECT_NEAR(saved.area_m2, 10000, 1);
  EXPECT_EQ(saved.created_s, 1000.0);

  // The file is GeoJSON that QGIS can open.
  const auto data = Json::parse(read("fields/" + saved.id + ".geojson"));
  EXPECT_EQ(data["type"], "FeatureCollection");
  const auto & ring = data["features"][0]["geometry"]["coordinates"][0];
  EXPECT_EQ(ring.front(), ring.back());
  EXPECT_EQ(ring.size(), 5u);
  EXPECT_EQ(data["features"][0]["properties"]["name"], "East field");
  EXPECT_EQ(data["features"][0]["properties"]["created"], "1970-01-01T00:16:40Z");
  EXPECT_EQ(data["features"][1]["properties"]["role"], "keep_out");

  saved.name = "East paddock";
  const Field updated = store.save(saved);
  EXPECT_EQ(updated.created_s, 1000.0);
  EXPECT_EQ(updated.modified_s, 2000.0);
  const auto listing = store.list();
  ASSERT_EQ(listing.fields.size(), 1u);
  EXPECT_EQ(listing.fields[0].name, "East paddock");
  EXPECT_TRUE(listing.problems.empty());
  ASSERT_EQ(listing.fields[0].boundary.size(), kSquare100.size());
  for (std::size_t i = 0; i < kSquare100.size(); ++i) {
    EXPECT_NEAR(listing.fields[0].boundary[i].lat, kSquare100[i].lat, 1e-12);
    EXPECT_NEAR(listing.fields[0].boundary[i].lon, kSquare100[i].lon, 1e-12);
  }
  EXPECT_EQ(listing.fields[0].keep_out.size(), 1u);
  EXPECT_EQ(listing.fields[0].created_s, 1000.0);

  EXPECT_TRUE(store.remove(saved.id));
  EXPECT_FALSE(store.remove(saved.id));
  EXPECT_TRUE(store.list().fields.empty());
}

TEST_F(TempDir, FieldStoreRefusesDuplicatesAndUnknownIds)
{
  FieldStore store(dir_);
  store.save(make_field());
  EXPECT_THROW(store.save(make_field()), FieldInvalid);
  Field other = make_field();
  other.id = "0123abcd-0000-0000-0000-000000000000";
  other.name = "Other";
  try {
    store.save(other);
    ADD_FAILURE();
  } catch (const FieldInvalid & e) {
    EXPECT_NE(std::string(e.what()).find("no longer exists"), std::string::npos);
  }
  EXPECT_THROW(store.remove("../../etc/passwd"), FieldInvalid);
}

TEST_F(TempDir, FieldStoreSkipsUnreadableFiles)
{
  FieldStore store(dir_);
  store.save(make_field());
  write("fields/broken.geojson", "{nope");
  const auto listing = store.list();
  EXPECT_EQ(listing.fields.size(), 1u);
  EXPECT_EQ(listing.problems, std::vector<std::string>{"broken.geojson"});
}

TEST_F(TempDir, FieldStoreSortsByNameIgnoringCase)
{
  FieldStore store(dir_);
  for (const char * name : {"beta", "Alpha", "charlie"}) {
    Field f = make_field();
    f.name = name;
    store.save(f);
  }
  std::vector<std::string> names;
  for (const auto & f : store.list().fields) {
    names.push_back(f.name);
  }
  EXPECT_EQ(names, (std::vector<std::string>{"Alpha", "beta", "charlie"}));
}

TEST_F(TempDir, ReadsAFieldThePythonNodeWrote)
{
  std::filesystem::create_directories(dir_ / "fields");
  write(
    "fields/0123abcd-1111-2222-3333-444455556666.geojson",
    R"({"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {
      "role": "boundary", "id": "0123abcd-1111-2222-3333-444455556666", "name": "North",
      "headland_width_m": 3.0, "source": "recorded", "notes": "", "area_m2": 10000.0,
      "created": "2026-11-01T10:00:00Z", "modified": "2026-11-02T10:00:00Z"},
      "geometry": {"type": "Polygon", "coordinates": [[[151.21, -33.86], [151.211, -33.86],
      [151.211, -33.859], [151.21, -33.86]]]}}]})");
  const auto listing = FieldStore(dir_).list();
  ASSERT_EQ(listing.fields.size(), 1u);
  const Field & f = listing.fields[0];
  EXPECT_EQ(f.name, "North");
  EXPECT_EQ(f.source, "recorded");
  EXPECT_EQ(f.headland_width_m, 3.0);
  EXPECT_EQ(f.boundary.size(), 3u);
  EXPECT_EQ(f.boundary[1].lon, 151.211);
  EXPECT_EQ(f.created_s, 1793527200.0);
}

TEST_F(TempDir, Home)
{
  FieldStore store(dir_);
  EXPECT_FALSE(store.get_home());
  store.set_home({-33.86, 151.21});
  const auto home = store.get_home();
  ASSERT_TRUE(home);
  EXPECT_EQ(home->lat, -33.86);
  EXPECT_EQ(home->lon, 151.21);
  EXPECT_THROW(store.set_home({123.0, 0.0}), FieldInvalid);
}

// ---- recording ----

TEST(Recording, FixQuality)
{
  EXPECT_EQ(fix_quality(-1, 2), kFixNone);
  EXPECT_EQ(fix_quality(0, std::nullopt), kFixGps);
  EXPECT_EQ(fix_quality(1, std::nullopt), kFixDgps);
  EXPECT_EQ(fix_quality(2, std::nullopt), kFixRtkFloat);
  EXPECT_EQ(fix_quality(2, 0), kFixDgps);
  EXPECT_EQ(fix_quality(2, 1), kFixRtkFloat);
  EXPECT_EQ(fix_quality(2, 2), kFixRtkFixed);
}

namespace
{

void drive(Recorder & r, const std::vector<XY> & points, int quality = kFixRtkFixed)
{
  for (const auto & p : points) {
    r.add_fix(kProj.to_latlon(p), quality);
  }
}

std::vector<XY> square_track(double step = 0.5)
{
  std::vector<double> side;
  for (int i = 0; i < static_cast<int>(20 / step); ++i) {
    side.push_back(i * step);
  }
  std::vector<XY> out;
  for (double x : side) {out.push_back({x, 0});}
  for (double y : side) {out.push_back({20, y});}
  for (double x : side) {out.push_back({20 - x, 20});}
  for (double y : side) {out.push_back({0, 20 - y});}
  out.push_back({0, 0});
  return out;
}

}  // namespace

TEST(Recording, RecordsAndSimplifiesASquare)
{
  Recorder r("rtk_fixed");
  EXPECT_EQ(r.state(), kIdle);
  r.start();
  drive(r, square_track());
  EXPECT_EQ(r.state(), kRecording);
  EXPECT_GT(r.points().size(), 100u);
  r.stop();
  EXPECT_EQ(r.state(), kFinished);
  EXPECT_EQ(r.points().size(), 4u);
  EXPECT_NEAR(ring_area_m2(kProj.to_xy(r.points())), 400, 4);
  EXPECT_NEAR(r.length_m(), 80, 1.6);
  EXPECT_EQ(r.message(), "Boundary recorded: 4 corners, 0.04 ha.");
}

TEST(Recording, SkipsPoorFixesAndClosePoints)
{
  Recorder r("rtk_fixed", 0.2);
  r.start();
  EXPECT_FALSE(r.add_fix(kProj.to_latlon({0, 0}), kFixRtkFloat));
  EXPECT_EQ(r.skipped(), 1u);
  EXPECT_TRUE(r.add_fix(kProj.to_latlon({0, 0}), kFixRtkFixed));
  EXPECT_FALSE(r.add_fix(kProj.to_latlon({0.1, 0}), kFixRtkFixed));  // too close
  EXPECT_TRUE(r.add_fix(kProj.to_latlon({0.3, 0}), kFixRtkFixed));
  Recorder idle("any");
  EXPECT_FALSE(idle.add_fix(kProj.to_latlon({0, 0}), kFixGps));  // not started
}

TEST(Recording, TooShortOrFlat)
{
  Recorder r("any");
  r.start();
  drive(r, {{0, 0}, {1, 0}}, kFixGps);
  r.stop();
  EXPECT_EQ(r.state(), kFinished);
  EXPECT_TRUE(r.points().empty());
  EXPECT_EQ(r.message(), "Too few positions were recorded to make a boundary.");
  r.start();
  std::vector<XY> line;
  for (int x = 0; x < 10; ++x) {
    line.push_back({static_cast<double>(x), 0});
  }
  drive(r, line, kFixGps);
  r.stop();
  EXPECT_TRUE(r.points().empty());
  EXPECT_EQ(r.message(), "The recorded track doesn't enclose an area.");
}

TEST(Recording, ReportsOpenGapAndClear)
{
  Recorder r("any");
  r.start();
  drive(r, {{0, 0}, {20, 0}, {20, 20}, {0, 20}, {0, 6}}, kFixGps);
  r.stop();
  EXPECT_NE(r.message().find("The end was 6.0 m from the start; the gap was closed"),
    std::string::npos);
  r.clear();
  EXPECT_EQ(r.state(), kIdle);
  EXPECT_TRUE(r.points().empty());
}

TEST(Recording, RejectsUnknownRequirement)
{
  EXPECT_THROW(Recorder("perfect"), std::invalid_argument);
}
