#include "r5_dashboard/fields.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <random>
#include <regex>
#include <sstream>

#include "r5_dashboard/files.hpp"

namespace r5_dashboard::fields
{

namespace fs = std::filesystem;

namespace
{

const char * const kSources[] = {"drawn", "recorded", "imported"};

double now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
         .count();
}

std::string iso(double t)
{
  const std::time_t seconds = static_cast<std::time_t>(t);
  std::tm tm {};
  gmtime_r(&seconds, &tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

double from_iso(const Json & value)
{
  if (!value.is_string()) {
    return 0.0;
  }
  std::tm tm {};
  const std::string text = value.get<std::string>();
  const char * end = strptime(text.c_str(), "%Y-%m-%dT%H:%M:%SZ", &tm);
  if (!end || *end != '\0') {
    return 0.0;
  }
  return static_cast<double>(timegm(&tm));
}

std::string uuid4()
{
  static std::random_device device;
  std::uniform_int_distribution<int> byte(0, 255);
  unsigned char b[16];
  for (auto & x : b) {
    x = static_cast<unsigned char>(byte(device));
  }
  b[6] = static_cast<unsigned char>((b[6] & 0x0F) | 0x40);  // Version 4.
  b[8] = static_cast<unsigned char>((b[8] & 0x3F) | 0x80);  // RFC 4122 variant.
  char out[37];
  std::snprintf(
    out, sizeof(out), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
    b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14],
    b[15]);
  return out;
}

bool valid_position(const LatLon & p)
{
  return std::isfinite(p.lat) && std::isfinite(p.lon) && std::abs(p.lat) <= 90 &&
         std::abs(p.lon) <= 180;
}

bool all_valid(const std::vector<LatLon> & points)
{
  return std::all_of(points.begin(), points.end(), valid_position);
}

std::string strip(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n\f\v");
  if (begin == std::string::npos) {
    return "";
  }
  return s.substr(begin, s.find_last_not_of(" \t\r\n\f\v") - begin + 1);
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::size_t utf8_length(const std::string & s)
{
  return static_cast<std::size_t>(
    std::count_if(s.begin(), s.end(), [](char c) {return (c & 0xC0) != 0x80;}));
}

Json ring_json(const std::vector<LatLon> & points)
{
  Json coords = Json::array();
  for (const auto & p : points) {
    coords.push_back({p.lon, p.lat});
  }
  if (!points.empty()) {
    coords.push_back(coords.front());  // GeoJSON rings repeat the first point.
  }
  return coords;
}

std::vector<LatLon> ring_from_feature(const Json & feature)
{
  Json coords = feature.at("geometry").at("coordinates").at(0);
  if (coords.size() > 1 && coords.front() == coords.back()) {
    coords.erase(coords.size() - 1);
  }
  std::vector<LatLon> out;
  for (const auto & c : coords) {
    if (!c.is_array() || c.size() < 2) {
      throw std::invalid_argument("A position needs a longitude and a latitude.");
    }
    out.push_back({c.at(1).get<double>(), c.at(0).get<double>()});
  }
  return out;
}

std::string role(const Json & feature)
{
  const auto props = feature.find("properties");
  if (props == feature.end() || !props->is_object()) {
    return "";
  }
  const auto r = props->find("role");
  return r != props->end() && r->is_string() ? r->get<std::string>() : "";
}

std::string text(const Json & value)
{
  return value.is_string() ? value.get<std::string>() : value.dump();
}

std::string format(const char * fmt, double value)
{
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), fmt, value);
  return buffer;
}

}  // namespace

// ---- GeoJSON ----

Json Field::to_geojson() const
{
  Json features = Json::array();
  features.push_back(
  {
    {"type", "Feature"},
    {"properties", {
        {"role", "boundary"},
        {"id", id},
        {"name", name},
        {"headland_width_m", headland_width_m},
        {"source", source},
        {"notes", notes},
        {"area_m2", std::round(area_m2 * 100.0) / 100.0},
        {"created", iso(created_s)},
        {"modified", iso(modified_s)}}},
    {"geometry", {{"type", "Polygon"}, {"coordinates", Json::array({ring_json(boundary)})}}},
  });
  for (std::size_t i = 0; i < keep_out.size(); ++i) {
    features.push_back(
    {
      {"type", "Feature"},
      {"properties", {{"role", "keep_out"}, {"index", i}}},
      {"geometry", {{"type", "Polygon"}, {"coordinates", Json::array({ring_json(keep_out[i])})}}},
    });
  }
  return {{"type", "FeatureCollection"}, {"features", features}};
}

Field Field::from_geojson(const Json & data)
{
  const Json & features = data.at("features");
  const Json * boundary = nullptr;
  for (const auto & f : features) {
    if (role(f) == "boundary") {
      boundary = &f;
      break;
    }
  }
  if (!boundary) {
    throw std::invalid_argument("No boundary.");
  }
  const Json & props = boundary->at("properties");
  Field field;
  field.id = text(props.at("id"));
  field.name = text(props.at("name"));
  field.boundary = ring_from_feature(*boundary);
  for (const auto & f : features) {
    if (role(f) == "keep_out") {
      field.keep_out.push_back(ring_from_feature(f));
    }
  }
  field.headland_width_m = props.value("headland_width_m", kDefaultHeadlandM);
  field.source = props.contains("source") ? text(props["source"]) : "imported";
  field.notes = props.contains("notes") ? text(props["notes"]) : "";
  field.area_m2 = props.value("area_m2", 0.0);
  field.created_s = from_iso(props.value("created", Json()));
  field.modified_s = from_iso(props.value("modified", Json()));
  return field;
}

// ---- Validation (spec FM-6) ----

double field_area_m2(const std::vector<LatLon> & boundary)
{
  const auto proj = geo::LocalProjection::around(boundary);
  return geo::ring_area_m2(proj.to_xy(boundary));
}

void validate(const Field & f, const std::vector<std::string> & other_names)
{
  const std::string name = strip(f.name);
  if (name.empty()) {
    throw FieldInvalid("Give the field a name.");
  }
  if (utf8_length(name) > kMaxNameLength) {
    throw FieldInvalid(
            "The name is too long (at most " + std::to_string(kMaxNameLength) + " characters).");
  }
  for (const auto & other : other_names) {
    if (lower(other) == lower(name)) {
      throw FieldInvalid("There is already a field called “" + name + "”.");
    }
  }
  if (std::none_of(std::begin(kSources), std::end(kSources), [&f](const char * s) {
      return f.source == s;
    }))
  {
    throw FieldInvalid("Unknown field source “" + f.source + "”.");
  }
  if (!(0.0 <= f.headland_width_m && f.headland_width_m <= kMaxHeadlandM) ||
    !std::isfinite(f.headland_width_m))
  {
    throw FieldInvalid(format("The headland width must be between 0 and %.0f m.", kMaxHeadlandM));
  }
  if (f.boundary.size() < 3) {
    throw FieldInvalid("The boundary needs at least 3 points.");
  }
  if (!all_valid(f.boundary) ||
    !std::all_of(f.keep_out.begin(), f.keep_out.end(), all_valid))
  {
    throw FieldInvalid("A point is outside the valid range of latitude and longitude.");
  }

  const auto proj = geo::LocalProjection::around(f.boundary);
  const geo::Ring outer = proj.to_xy(f.boundary);
  if (geo::ring_self_intersects(outer)) {
    throw FieldInvalid("The boundary crosses itself.");
  }
  if (geo::ring_area_m2(outer) < 1.0) {
    throw FieldInvalid("The boundary encloses no area.");
  }
  std::vector<geo::Ring> zones;
  for (std::size_t i = 0; i < f.keep_out.size(); ++i) {
    const std::string n = std::to_string(i + 1);
    if (f.keep_out[i].size() < 3) {
      throw FieldInvalid("Keep-out zone " + n + " needs at least 3 points.");
    }
    const geo::Ring xy = proj.to_xy(f.keep_out[i]);
    if (geo::ring_self_intersects(xy)) {
      throw FieldInvalid("Keep-out zone " + n + " crosses itself.");
    }
    if (!geo::ring_inside(xy, outer)) {
      throw FieldInvalid("Keep-out zone " + n + " must lie inside the boundary.");
    }
    zones.push_back(xy);
  }
  for (std::size_t i = 0; i < zones.size(); ++i) {
    for (std::size_t j = i + 1; j < zones.size(); ++j) {
      if (geo::rings_cross(zones[i], zones[j])) {
        throw FieldInvalid(
                "Keep-out zones " + std::to_string(i + 1) + " and " + std::to_string(j + 1) +
                " overlap. Merge them into one.");
      }
    }
  }
}

// ---- Storage (spec FM-8, 6.2, 6.3) ----

FieldStore::FieldStore(const fs::path & data_dir, Clock clock)
: dir_(data_dir / "fields"), home_path_(data_dir / "home.geojson"),
  clock_(clock ? std::move(clock) : Clock(now_s))
{
}

fs::path FieldStore::path(const std::string & id) const
{
  static const std::regex kIdPattern("^[0-9a-f-]{8,64}$");
  if (!std::regex_match(id, kIdPattern)) {
    throw FieldInvalid("Unknown field.");
  }
  return dir_ / (id + ".geojson");
}

Listing FieldStore::list() const
{
  Listing out;
  std::error_code ec;
  if (!fs::is_directory(dir_, ec)) {
    return out;
  }
  std::vector<fs::path> paths;
  for (const auto & entry : fs::directory_iterator(dir_, ec)) {
    if (entry.path().extension() == ".geojson" && entry.path().filename().string()[0] != '.') {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  for (const auto & p : paths) {
    try {
      const auto contents = read_file(p);
      if (!contents) {
        continue;  // Deleted since it was listed.
      }
      out.fields.push_back(Field::from_geojson(Json::parse(*contents)));
    } catch (const std::exception &) {
      out.problems.push_back(p.filename().string());
    }
  }
  std::stable_sort(out.fields.begin(), out.fields.end(), [](const Field & a, const Field & b) {
      return lower(a.name) < lower(b.name);
    });
  return out;
}

Field FieldStore::save(Field f) const
{
  const auto existing = list().fields;
  double created = 0.0;
  if (!f.id.empty()) {
    const auto previous = std::find_if(existing.begin(), existing.end(), [&f](const Field & e) {
          return e.id == f.id;
        });
    if (previous == existing.end()) {
      throw FieldInvalid(
              "That field no longer exists. It may have been deleted on another device.");
    }
    created = previous->created_s;
  } else {
    f.id = uuid4();
    created = clock_();
  }
  f.name = strip(f.name);
  std::vector<std::string> others;
  for (const auto & e : existing) {
    if (e.id != f.id) {
      others.push_back(e.name);
    }
  }
  validate(f, others);
  f.area_m2 = field_area_m2(f.boundary);
  f.created_s = created;
  f.modified_s = clock_();
  write_private_file(path(f.id), f.to_geojson().dump(2) + "\n");
  return f;
}

bool FieldStore::remove(const std::string & id) const
{
  return fs::remove(path(id));
}

std::optional<LatLon> FieldStore::get_home() const
{
  try {
    const auto contents = read_file(home_path_);
    if (!contents) {
      return std::nullopt;
    }
    const Json data = Json::parse(*contents);
    const Json & c = data.at("features").at(0).at("geometry").at("coordinates");
    return LatLon{c.at(1).get<double>(), c.at(0).get<double>()};
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void FieldStore::set_home(const LatLon & position) const
{
  if (!valid_position(position)) {
    throw FieldInvalid("That position is outside the valid range.");
  }
  const Json data = {
    {"type", "FeatureCollection"},
    {"features", Json::array({{
        {"type", "Feature"},
        {"properties", {{"role", "home"}, {"set", iso(clock_())}}},
        {"geometry", {{"type", "Point"}, {"coordinates", {position.lon, position.lat}}}},
      }})},
  };
  write_private_file(home_path_, data.dump(2) + "\n");
}

// ---- Boundary recording (spec FM-4) ----

const char * fix_name(int quality)
{
  switch (quality) {
    case kFixGps: return "3D fix";
    case kFixDgps: return "Corrected (SBAS)";
    case kFixRtkFloat: return "RTK float";
    case kFixRtkFixed: return "RTK fixed";
    default: return "No fix";
  }
}

std::optional<int> requirement(const std::string & name)
{
  if (name == "any") {
    return kFixGps;
  }
  if (name == "rtk_float") {
    return kFixRtkFloat;
  }
  if (name == "rtk_fixed") {
    return kFixRtkFixed;
  }
  return std::nullopt;
}

int fix_quality(int navsat_status, std::optional<int> carrier)
{
  if (navsat_status < 0) {
    return kFixNone;
  }
  if (carrier == 2) {
    return kFixRtkFixed;
  }
  if (carrier == 1) {
    return kFixRtkFloat;
  }
  if (navsat_status == 2) {
    // GBAS without NavPVT: corrections are in use, but float and fixed can't be told apart.
    return carrier ? kFixDgps : kFixRtkFloat;
  }
  if (navsat_status == 1) {
    return kFixDgps;
  }
  return kFixGps;
}

Recorder::Recorder(const std::string & required, double min_spacing_m, double tolerance_m,
  Clock clock)
: min_spacing_m_(min_spacing_m), tolerance_m_(tolerance_m),
  clock_(clock ? std::move(clock) : Clock(now_s))
{
  const auto quality = requirement(required);
  if (!quality) {
    throw std::invalid_argument(
            "The fix requirement must be one of any, rtk_fixed, rtk_float, not " + required + ".");
  }
  min_quality_ = *quality;
}

void Recorder::reset()
{
  state_ = kIdle;
  points_.clear();
  skipped_ = 0;
  length_m_ = 0.0;
  current_quality_ = kFixNone;
  message_.clear();
  started_s_ = 0.0;
  projection_.reset();
}

void Recorder::start()
{
  reset();
  state_ = kRecording;
  started_s_ = clock_();
  message_ = "Drive slowly around the edge of the field.";
}

bool Recorder::add_fix(const LatLon & position, int quality)
{
  current_quality_ = quality;
  if (state_ != kRecording) {
    return false;
  }
  if (quality < min_quality_ || !valid_position(position)) {
    ++skipped_;
    return false;
  }
  if (!projection_) {
    projection_.emplace(position);
  }
  if (!points_.empty()) {
    const double step =
      geo::distance(projection_->to_xy(points_.back()), projection_->to_xy(position));
    if (step < min_spacing_m_) {
      return false;
    }
    length_m_ += step;
  }
  points_.push_back(position);
  return true;
}

void Recorder::stop()
{
  if (state_ != kRecording) {
    return;
  }
  state_ = kFinished;
  if (points_.size() < 3 || !projection_) {
    message_ = "Too few positions were recorded to make a boundary.";
    points_.clear();
    return;
  }
  const auto track = projection_->to_xy(points_);
  const geo::Ring ring = geo::close_track(track, tolerance_m_);
  if (ring.size() < 3 || geo::ring_area_m2(ring) < 1.0) {
    message_ = "The recorded track doesn't enclose an area.";
    points_.clear();
    return;
  }
  const double gap = geo::distance(track.front(), track.back());
  points_.clear();
  for (const auto & p : ring) {
    points_.push_back(projection_->to_latlon(p));
  }
  length_m_ = geo::path_length_m(track);
  message_ = "Boundary recorded: " + std::to_string(points_.size()) + " corners, " +
    format("%.2f ha.", geo::ring_area_m2(ring) / 10000.0);
  if (gap > 2.0) {
    message_ += format(
      " The end was %.1f m from the start; the gap was closed with a straight line.", gap);
  }
  if (geo::ring_self_intersects(ring)) {
    message_ += " The track crosses itself; fix it in the editor before saving.";
  }
}

void Recorder::clear()
{
  if (state_ != kRecording) {
    reset();
  }
}

}  // namespace r5_dashboard::fields
