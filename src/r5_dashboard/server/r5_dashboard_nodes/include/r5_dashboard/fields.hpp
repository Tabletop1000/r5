#ifndef R5_DASHBOARD__FIELDS_HPP_
#define R5_DASHBOARD__FIELDS_HPP_

#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "r5_dashboard/geo.hpp"

// Field storage, validation and boundary recording (spec 4.1, 6.3, FM-4, FM-6, FM-8, FM-10),
// without ROS. field_manager_node.cpp connects it to ROS.
//
// Each field is one GeoJSON file, fields/<id>.geojson: a FeatureCollection with the boundary as
// the first feature (carrying the field's properties) and one feature per keep-out zone. The files
// open directly in QGIS. job_manager, still Python, reads them with fields.py.
namespace r5_dashboard::fields
{

using geo::LatLon;
using Json = nlohmann::ordered_json;
using Clock = std::function<double()>;  // Seconds since the epoch.

constexpr std::size_t kMaxNameLength = 60;
constexpr double kMaxHeadlandM = 20.0;
constexpr double kDefaultHeadlandM = 2.0;

struct Field
{
  std::string id;
  std::string name;
  std::vector<LatLon> boundary;
  std::vector<std::vector<LatLon>> keep_out;
  double headland_width_m = kDefaultHeadlandM;
  std::string source = "drawn";
  std::string notes;
  double area_m2 = 0.0;
  double created_s = 0.0;
  double modified_s = 0.0;

  Json to_geojson() const;
  // Throws on anything that isn't a field file.
  static Field from_geojson(const Json & data);
};

// A field or home point that can't be saved. The message is shown to the user.
class FieldInvalid : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

double field_area_m2(const std::vector<LatLon> & boundary);
// Throws FieldInvalid with a plain-language reason if the field can't be saved.
void validate(const Field & field, const std::vector<std::string> & other_names);

struct Listing
{
  std::vector<Field> fields;  // Sorted by name.
  std::vector<std::string> problems;  // Files that couldn't be read.
};

class FieldStore
{
public:
  explicit FieldStore(const std::filesystem::path & data_dir, Clock clock = {});

  const std::filesystem::path & dir() const {return dir_;}
  Listing list() const;
  // Validates and writes. A new field (empty id) gets an id. Returns the field as stored. Throws
  // FieldInvalid, or std::filesystem::filesystem_error if it can't be written.
  Field save(Field field) const;
  // False if there was no such field.
  bool remove(const std::string & id) const;

  std::optional<LatLon> get_home() const;
  void set_home(const LatLon & position) const;

private:
  std::filesystem::path path(const std::string & id) const;

  std::filesystem::path dir_;
  std::filesystem::path home_path_;
  Clock clock_;
};

// ---- Boundary recording (spec FM-4) ----

enum RecorderState : int { kIdle = 0, kRecording = 1, kFinished = 2 };

// Fix qualities, worst to best.
enum FixQuality : int { kFixNone = 0, kFixGps, kFixDgps, kFixRtkFloat, kFixRtkFixed };
const char * fix_name(int quality);
// The quality a requirement ("any", "rtk_float" or "rtk_fixed") asks for, or empty.
std::optional<int> requirement(const std::string & name);

// Fix quality from NavSatFix status.status and, when known, the u-blox NavPVT carrier solution
// (0 none, 1 RTK float, 2 RTK fixed).
int fix_quality(int navsat_status, std::optional<int> carrier);

// Collects positions while the robot is driven around a field edge.
//
// Points are kept only when the fix is good enough and the robot has moved min_spacing_m from the
// last kept point. Stopping closes the track and simplifies it to within tolerance_m.
class Recorder
{
public:
  // Throws std::invalid_argument for an unknown requirement.
  explicit Recorder(
    const std::string & required = "rtk_fixed", double min_spacing_m = 0.2,
    double tolerance_m = 0.1, Clock clock = {});

  void start();
  // Offers a position. Returns true if it was kept.
  bool add_fix(const LatLon & position, int quality);
  // Finishes: closes and simplifies the track. Unusable tracks finish with an explanation.
  void stop();
  void clear();

  int state() const {return state_;}
  const std::vector<LatLon> & points() const {return points_;}
  unsigned skipped() const {return skipped_;}
  double length_m() const {return length_m_;}
  int min_quality() const {return min_quality_;}
  int current_quality() const {return current_quality_;}
  const std::string & message() const {return message_;}
  double started_s() const {return started_s_;}

private:
  void reset();

  int min_quality_;
  double min_spacing_m_;
  double tolerance_m_;
  Clock clock_;
  int state_ = kIdle;
  std::vector<LatLon> points_;
  unsigned skipped_ = 0;
  double length_m_ = 0.0;
  int current_quality_ = kFixNone;
  std::string message_;
  double started_s_ = 0.0;
  std::optional<geo::LocalProjection> projection_;
};

}  // namespace r5_dashboard::fields

#endif  // R5_DASHBOARD__FIELDS_HPP_
