#include "r5_dashboard/settings.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

#include "r5_dashboard/files.hpp"

namespace r5_dashboard::settings
{

namespace fs = std::filesystem;

namespace
{

Setting number(
  std::string key, std::string label, std::string group, double default_value,
  std::string unit, double minimum, double maximum, double step, std::string help)
{
  Setting s;
  s.key = std::move(key);
  s.label = std::move(label);
  s.group = std::move(group);
  s.kind = "number";
  s.default_value = default_value;
  s.unit = std::move(unit);
  s.minimum = minimum;
  s.maximum = maximum;
  s.step = step;
  s.help = std::move(help);
  return s;
}

Setting choice(
  std::string key, std::string label, std::string group, std::string default_value,
  Choices choices, std::string help)
{
  Setting s;
  s.key = std::move(key);
  s.label = std::move(label);
  s.group = std::move(group);
  s.kind = "choice";
  s.default_value = std::move(default_value);
  s.choices = std::move(choices);
  s.help = std::move(help);
  return s;
}

Setting speed(
  std::string key, std::string label, double default_value, std::string help)
{
  Setting s = number(
    std::move(key), std::move(label), "Driving", default_value, "km/h", 0.1,
    kHardwareMaxSpeedKmh, 0.01, std::move(help));
  s.quantity = "speed";
  return s;
}

std::vector<Setting> make_settings()
{
  std::vector<Setting> out;

  Setting name;
  name.key = "robot_name";
  name.label = "Nickname";
  name.group = "Robot";
  name.kind = "text";
  name.default_value = "R5";
  name.max_length = 20;
  name.help = "Shown at the top of the screen and next to the robot on the map.";
  out.push_back(name);

  out.push_back(
    choice(
      "units", "Units", "Display", kUnitsMetric,
      {{kUnitsMetric, "Metric (km/h, hectares, litres)"},
        {kUnitsUs, "US (mph, acres, US gallons, feet)"}},
      "For speeds, areas, volumes and distances on the dashboard. Sensor readings stay in SI "
      "units."));
  out.push_back(
    choice(
      "time_zone", "Time zone", "Display", kDeviceTimeZone, time_zone_choices(),
      "For times and dates on the dashboard."));
  out.push_back(
    speed(
      "max_speed_kmh", "Maximum speed", kHardwareMaxSpeedKmh,
      "No part of a spray job goes faster than this. The Xbox controller has its own speeds."));
  out.push_back(
    speed(
      "transit_speed_kmh", "Transit speed", kHardwareMaxSpeedKmh,
      "Driving to the field, home and back, with the sprayer off. Never more than the maximum "
      "speed."));
  out.push_back(
    speed(
      "turn_speed_kmh", "Turn speed", 0.5,
      "Turning between rows. Never more than the maximum speed."));
  out.push_back(
    choice(
      "comms_loss_action", "If the link is lost", "Link loss", kCommsContinue,
      {{kCommsContinue, "Keep working"},
        {kCommsReturnHome, "Return home"},
        {kCommsDelayedReturnHome, "Keep working, then return home"},
        {kCommsStop, "Stop and wait"}},
      "What a running spray job does when no dashboard has been connected for the time below. "
      "The sprayer is off whenever the robot stops or drives home. After returning home or "
      "stopping, the job waits for Resume."));
  out.push_back(
    number(
      "comms_loss_timeout_s", "Link counts as lost after", "Link loss", 30.0, "s", 15.0, 600.0,
      1.0,
      "Shorter dropouts are ignored. It takes about 15 s to notice a dashboard has gone. "
      "A phone with its screen locked usually counts as disconnected."));
  Setting delay = number(
    "comms_loss_delay_min", "Return home after", "Link loss", 5.0, "min", 1.0, 120.0, 1.0,
    "How long the robot keeps working without a link before it drives home.");
  delay.shown_when = std::make_pair(std::string("comms_loss_action"),
      std::string(kCommsDelayedReturnHome));
  out.push_back(delay);
  out.push_back(
    choice(
      "low_battery_action", "If the battery is low", "Battery", kBatteryReturnHome,
      {{kBatteryWarn, "Only warn"},
        {kBatteryReturnHome, "Return home"},
        {kBatteryStop, "Stop and wait"}},
      "What a running spray job does when the battery drops below the level below. There's "
      "always an alert. Resume needs the battery above the level again. Nothing happens until "
      "the robot reports its battery level."));
  out.push_back(
    number(
      "low_battery_percent", "Low battery level", "Battery", 20.0, "%", 5.0, 80.0, 1.0,
      "Counts as low after 10 s below this, so a short dip under load doesn't."));
  out.push_back(
    number(
      "slope_limit_deg", "Slope limit", "Slope", 15.0, "°", 5.0, 45.0, 1.0,
      "A spray job stops if the robot tilts more than this for a second, from the IMU. After "
      "Resume it doesn't stop again until it has been on level ground, so it can drive off."));
  return out;
}

// printf's %g, as Python's format(x, "g").
std::string g(double v)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%g", v);
  return buffer;
}

// Python's repr() of a JSON value, for messages.
std::string repr(const Json & value)
{
  if (value.is_string()) {
    return "'" + value.get<std::string>() + "'";
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "True" : "False";
  }
  if (value.is_null()) {
    return "None";
  }
  return value.dump();
}

std::size_t utf8_length(const std::string & s)
{
  return static_cast<std::size_t>(
    std::count_if(s.begin(), s.end(), [](char c) {return (c & 0xC0) != 0x80;}));
}

// Python's " ".join(value.split()), for ASCII whitespace.
std::string collapse_whitespace(const std::string & s)
{
  std::istringstream in(s);
  std::string word;
  std::string out;
  while (in >> word) {
    out += (out.empty() ? "" : " ") + word;
  }
  return out;
}

bool is_tzif(const fs::path & path)
{
  std::ifstream in(path, std::ios::binary);
  char magic[4] = {};
  return in.read(magic, 4) && std::string(magic, 4) == "TZif";
}

}  // namespace

Json Setting::to_json() const
{
  Json out = {
    {"key", key}, {"label", label}, {"group", group}, {"kind", kind},
    {"default", default_value}, {"help", help}, {"unit", unit}};
  if (minimum) {
    out["minimum"] = *minimum;
  }
  if (maximum) {
    out["maximum"] = *maximum;
  }
  if (step) {
    out["step"] = *step;
  }
  if (max_length) {
    out["max_length"] = *max_length;
  }
  if (!choices.empty()) {
    Json list = Json::array();
    for (const auto & [value, choice_label] : choices) {
      list.push_back({{"value", value}, {"label", choice_label}});
    }
    out["choices"] = list;
  }
  if (shown_when) {
    out["shown_when"] = {{"key", shown_when->first}, {"value", shown_when->second}};
  }
  if (!quantity.empty()) {
    out["quantity"] = quantity;
  }
  return out;
}

Choices time_zone_choices(const fs::path & zoneinfo_dir)
{
  // As Python's zoneinfo.available_timezones(): every TZif file, skipping the "posix" and "right"
  // copies, then only the Area/Location names.
  std::vector<std::string> names;
  std::error_code ec;
  fs::recursive_directory_iterator it(zoneinfo_dir, ec), end;
  for (; !ec && it != end; it.increment(ec)) {
    const auto relative = it->path().lexically_relative(zoneinfo_dir).string();
    if (it->is_directory(ec)) {
      if (relative == "posix" || relative == "right") {
        it.disable_recursion_pending();
      }
      continue;
    }
    if (relative.find('/') == std::string::npos) {
      continue;
    }
    if (relative.rfind("Etc/", 0) == 0 || relative.rfind("SystemV/", 0) == 0) {
      continue;
    }
    if (is_tzif(it->path())) {
      names.push_back(relative);
    }
  }
  std::sort(names.begin(), names.end());
  Choices out = {{kDeviceTimeZone, "This device's time zone"}, {"UTC", "UTC"}};
  for (const auto & name : names) {
    std::string label = name;
    std::replace(label.begin(), label.end(), '_', ' ');
    out.emplace_back(name, label);
  }
  return out;
}

const std::vector<Setting> & all()
{
  static const std::vector<Setting> settings = make_settings();
  return settings;
}

const Setting * find(const std::string & key)
{
  for (const auto & s : all()) {
    if (s.key == key) {
      return &s;
    }
  }
  return nullptr;
}

Json defaults()
{
  Json values = Json::object();
  for (const auto & s : all()) {
    values[s.key] = s.default_value;
  }
  return values;
}

std::string schema_json()
{
  Json list = Json::array();
  for (const auto & s : all()) {
    list.push_back(s.to_json());
  }
  return list.dump();
}

Json check_value(const Setting & setting, const Json & value)
{
  if (setting.kind == "number") {
    if (!value.is_number() || !std::isfinite(value.get<double>())) {
      throw SettingsError(setting.label + " must be a number.");
    }
    const double v = value.get<double>();
    const std::string unit = setting.unit.empty() ? "" : " " + setting.unit;
    if (setting.minimum && v < *setting.minimum) {
      throw SettingsError(setting.label + " must be at least " + g(*setting.minimum) + unit + ".");
    }
    if (setting.maximum && v > *setting.maximum) {
      throw SettingsError(setting.label + " can be at most " + g(*setting.maximum) + unit + ".");
    }
    return v;
  }
  if (setting.kind == "text") {
    if (!value.is_string()) {
      throw SettingsError(setting.label + " must be text.");
    }
    const std::string v = collapse_whitespace(value.get<std::string>());
    if (v.empty()) {
      throw SettingsError(setting.label + " can't be empty.");
    }
    if (setting.max_length && utf8_length(v) > static_cast<std::size_t>(*setting.max_length)) {
      throw SettingsError(
              setting.label + " can be at most " + std::to_string(*setting.max_length) +
              " characters.");
    }
    return v;
  }
  if (setting.kind == "choice") {
    const bool known = value.is_string() &&
      std::any_of(setting.choices.begin(), setting.choices.end(), [&value](const auto & c) {
          return c.first == value.get<std::string>();
        });
    if (!known) {
      throw SettingsError(repr(value) + " isn't one of the choices for " + setting.label + ".");
    }
    return value;
  }
  throw SettingsError(setting.label + " has an unknown kind '" + setting.kind + "'.");
}

Json check_changes(const Json & changes)
{
  if (!changes.is_object()) {
    throw SettingsError("Settings must be sent as a JSON object.");
  }
  Json out = Json::object();
  for (const auto & [key, value] : changes.items()) {
    const Setting * setting = find(key);
    if (!setting) {
      throw SettingsError("There's no setting called '" + key + "'.");
    }
    out[key] = check_value(*setting, value);
  }
  return out;
}

SettingsStore::SettingsStore(const fs::path & data_dir)
: path_(data_dir / "settings.json")
{
}

Loaded SettingsStore::load() const
{
  Loaded loaded{defaults(), {}};
  Json saved;
  try {
    const auto text = read_file(path_);
    if (!text) {
      return loaded;
    }
    saved = Json::parse(*text);
  } catch (const std::exception &) {
    loaded.ignored.push_back(path_.string());
    return loaded;
  }
  if (!saved.is_object()) {
    loaded.ignored.push_back(path_.string());
    return loaded;
  }
  for (const auto & [key, value] : saved.items()) {
    const Setting * setting = find(key);
    if (!setting) {
      continue;  // A setting that has since been removed.
    }
    try {
      loaded.values[key] = check_value(*setting, value);
    } catch (const SettingsError &) {
      loaded.ignored.push_back(key);
    }
  }
  return loaded;
}

Json SettingsStore::update(const Json & current, const Json & changes) const
{
  Json values = current;
  const Json checked = check_changes(changes);
  for (const auto & [key, value] : checked.items()) {
    values[key] = value;
  }
  write_private_file(path_, values.dump(2) + "\n");
  return values;
}

}  // namespace r5_dashboard::settings
