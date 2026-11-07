#include <gtest/gtest.h>

#include <unistd.h>

#include <cstring>

#include <filesystem>
#include <fstream>
#include <string>

#include "r5_monitor/system_info.hpp"

namespace fs = std::filesystem;
using r5_monitor::git_version;
using r5_monitor::parse_wireless;
using r5_monitor::read_temperature_c;

namespace
{

// A Raspberry Pi 4's /proc/net/wireless.
const char * kWireless =
  "Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
  " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
  "wlan0: 0000   54.  -56.  -256        0      0      0      0     16        0\n"
  "wlan1: 0000   30.  -80.  -256        0      0      0      0      0        0\n";

class TempDir : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / ("r5_monitor_test_" + std::to_string(::getpid()));
    fs::create_directories(dir_);
  }
  void TearDown() override {fs::remove_all(dir_);}

  void write(const fs::path & relative, const std::string & text)
  {
    fs::create_directories((dir_ / relative).parent_path());
    std::ofstream(dir_ / relative) << text;
  }

  fs::path dir_;
};

}  // namespace

TEST(Wireless, TakesTheFirstInterface)
{
  const auto wifi = parse_wireless(kWireless, "");
  ASSERT_TRUE(wifi);
  EXPECT_EQ(wifi->interface, "wlan0");
  EXPECT_DOUBLE_EQ(wifi->dbm, -56.0);
}

TEST(Wireless, TakesTheNamedInterface)
{
  const auto wifi = parse_wireless(kWireless, "wlan1");
  ASSERT_TRUE(wifi);
  EXPECT_EQ(wifi->interface, "wlan1");
  EXPECT_DOUBLE_EQ(wifi->dbm, -80.0);
  EXPECT_FALSE(parse_wireless(kWireless, "wlan2"));
}

TEST(Wireless, UnsignedLevels)
{
  const std::string text = std::string(kWireless, std::strchr(kWireless, '\n') + 1) +
    " face | tus | link level noise\n"
    "wlp2s0: 0000   60.  200.  0\n";
  const auto wifi = parse_wireless(text, "");
  ASSERT_TRUE(wifi);
  EXPECT_EQ(wifi->interface, "wlp2s0");
  EXPECT_DOUBLE_EQ(wifi->dbm, -56.0);
}

TEST(Wireless, NoInterfaces)
{
  const std::string headers(kWireless, std::strstr(kWireless, "wlan0"));
  EXPECT_FALSE(parse_wireless(headers, ""));
  EXPECT_FALSE(parse_wireless("", ""));
}

TEST_F(TempDir, TemperatureIsTheHottestZone)
{
  write("thermal_zone0/temp", "48312\n");
  write("thermal_zone1/temp", "51500\n");
  write("thermal_zone2/temp", "not a number\n");
  write("cooling_device0/temp", "99000\n");
  const auto temp = read_temperature_c(dir_);
  ASSERT_TRUE(temp);
  EXPECT_DOUBLE_EQ(*temp, 51.5);
}

TEST_F(TempDir, NoTemperatureSensor)
{
  EXPECT_FALSE(read_temperature_c(dir_));
  EXPECT_FALSE(read_temperature_c(dir_ / "missing"));
}

TEST_F(TempDir, GitVersionOutsideARepository)
{
  EXPECT_EQ(git_version({dir_}), "unknown");
}

TEST_F(TempDir, GitVersionFromAParentDirectory)
{
  const auto repo = dir_ / "repo";
  fs::create_directories(repo / "install" / "r5_monitor");
  const std::string git = "git -c user.name=t -c user.email=t@t -C " + repo.string();
  ASSERT_EQ(
    std::system((git + " init -q && " + git + " commit -q --allow-empty -m x && " + git +
    " tag v1.2").c_str()), 0);
  EXPECT_EQ(git_version({repo / "install" / "r5_monitor"}), "v1.2");
}
