#include <gtest/gtest.h>

#include <sys/stat.h>

#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "r5_dashboard/settings.hpp"
#include "temp_dir.hpp"

namespace s = r5_dashboard::settings;
using s::Json;

TEST(Settings, EveryDefaultIsValid)
{
  EXPECT_EQ(s::check_changes(s::defaults()), s::defaults());
}

TEST(Settings, KeysAreUniqueAndShownWhenRefersToAChoice)
{
  std::set<std::string> keys;
  for (const auto & setting : s::all()) {
    EXPECT_TRUE(keys.insert(setting.key).second) << setting.key;
    if (setting.shown_when) {
      const auto * other = s::find(setting.shown_when->first);
      ASSERT_NE(other, nullptr);
      EXPECT_EQ(other->kind, "choice");
      bool found = false;
      for (const auto & c : other->choices) {
        found |= c.first == setting.shown_when->second;
      }
      EXPECT_TRUE(found);
    }
  }
}

TEST(Settings, SchemaIsJsonThePageCanRead)
{
  const auto schema = Json::parse(s::schema_json());
  Json by_key = Json::object();
  for (const auto & item : schema) {
    by_key[item["key"].get<std::string>()] = item;
  }
  EXPECT_EQ(by_key["max_speed_kmh"]["maximum"], s::kHardwareMaxSpeedKmh);
  EXPECT_EQ(by_key["max_speed_kmh"]["quantity"], "speed");
  const Json stop = {{"value", s::kCommsStop}, {"label", "Stop and wait"}};
  const auto & choices = by_key["comms_loss_action"]["choices"];
  EXPECT_NE(std::find(choices.begin(), choices.end(), stop), choices.end());
  EXPECT_EQ(
    by_key["comms_loss_delay_min"]["shown_when"],
    Json({{"key", "comms_loss_action"}, {"value", "delayed_return_home"}}));
  EXPECT_EQ(by_key["robot_name"]["max_length"], 20);
  EXPECT_FALSE(by_key["robot_name"].contains("minimum"));
}

TEST(Settings, BadValuesAreRefused)
{
  const std::vector<std::pair<Json, std::string>> cases = {
    {{{"max_speed_kmh", 5.0}}, "Maximum speed can be at most 2.16 km/h."},
    {{{"max_speed_kmh", 0.0}}, "Maximum speed must be at least 0.1 km/h."},
    {{{"max_speed_kmh", "fast"}}, "Maximum speed must be a number."},
    {{{"max_speed_kmh", true}}, "Maximum speed must be a number."},
    {{{"robot_name", "   "}}, "Nickname can't be empty."},
    {{{"robot_name", std::string(21, 'x')}}, "Nickname can be at most 20 characters."},
    {{{"robot_name", 5}}, "Nickname must be text."},
    {{{"comms_loss_action", "panic"}}, "'panic' isn't one of the choices for If the link is lost."},
    {{{"colour", "red"}}, "There's no setting called 'colour'."},
    {Json::array(), "Settings must be sent as a JSON object."},
  };
  for (const auto & [changes, message] : cases) {
    try {
      s::check_changes(changes);
      ADD_FAILURE() << "accepted " << changes.dump();
    } catch (const s::SettingsError & e) {
      EXPECT_EQ(e.what(), message);
    }
  }
}

TEST(Settings, LengthCountsCharactersNotBytes)
{
  // 20 characters, 40 bytes.
  std::string name;
  for (int i = 0; i < 20; ++i) {
    name += "é";
  }
  EXPECT_EQ(s::check_changes({{"robot_name", name}})["robot_name"], name);
}

TEST(Settings, TextIsTidiedAndNumbersBecomeFloats)
{
  const auto out = s::check_changes({{"robot_name", "  Big\tRed\n "}, {"comms_loss_timeout_s", 60}});
  EXPECT_EQ(out["robot_name"], "Big Red");
  EXPECT_TRUE(out["comms_loss_timeout_s"].is_number_float());
  EXPECT_EQ(out.dump(), "{\"robot_name\":\"Big Red\",\"comms_loss_timeout_s\":60.0}");
}

TEST_F(TempDir, StoreSavesAndLoads)
{
  s::SettingsStore store(dir_);
  auto loaded = store.load();
  EXPECT_EQ(loaded.values, s::defaults());
  EXPECT_TRUE(loaded.ignored.empty());
  const auto values =
    store.update(loaded.values, {{"robot_name", "Spot"}, {"comms_loss_action", s::kCommsStop}});
  const auto again = s::SettingsStore(dir_).load();
  EXPECT_EQ(again.values, values);
  EXPECT_TRUE(again.ignored.empty());
  EXPECT_EQ(values["robot_name"], "Spot");
  EXPECT_EQ(values["max_speed_kmh"], s::kHardwareMaxSpeedKmh);
  struct stat st {};
  ASSERT_EQ(::stat((dir_ / "settings.json").c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0600u);
}

TEST_F(TempDir, ABadValueChangesNothing)
{
  s::SettingsStore store(dir_);
  const auto values = store.load().values;
  EXPECT_THROW(store.update(values, {{"robot_name", "Spot"}, {"max_speed_kmh", 99.0}}),
    s::SettingsError);
  EXPECT_EQ(store.load().values, s::defaults());
}

TEST_F(TempDir, StoreFallsBackToDefaultsForBadSavedValues)
{
  write("settings.json", R"({"robot_name": "", "max_speed_kmh": 0.5, "old_setting": 1})");
  const auto loaded = s::SettingsStore(dir_).load();
  EXPECT_EQ(loaded.values["robot_name"], "R5");
  EXPECT_EQ(loaded.values["max_speed_kmh"], 0.5);
  EXPECT_EQ(loaded.ignored, std::vector<std::string>{"robot_name"});
}

TEST_F(TempDir, StoreSurvivesADamagedFile)
{
  write("settings.json", "{not json");
  const auto loaded = s::SettingsStore(dir_).load();
  EXPECT_EQ(loaded.values, s::defaults());
  EXPECT_EQ(loaded.ignored.size(), 1u);
}

TEST_F(TempDir, TimeZonesAreTzifFilesWithAnArea)
{
  std::filesystem::create_directories(dir_ / "Europe");
  std::filesystem::create_directories(dir_ / "America" / "Argentina");
  std::filesystem::create_directories(dir_ / "Etc");
  std::filesystem::create_directories(dir_ / "posix" / "Europe");
  write("Europe/London", "TZif2...");
  write("America/Argentina/Buenos_Aires", "TZif2...");
  write("Etc/GMT", "TZif2...");
  write("posix/Europe/London", "TZif2...");
  write("UTC", "TZif2...");
  write("Europe/README", "not a zone");
  const auto zones = s::time_zone_choices(dir_);
  const s::Choices expected = {
    {"device", "This device's time zone"}, {"UTC", "UTC"},
    {"America/Argentina/Buenos_Aires", "America/Argentina/Buenos Aires"},
    {"Europe/London", "Europe/London"}};
  EXPECT_EQ(zones, expected);
}
