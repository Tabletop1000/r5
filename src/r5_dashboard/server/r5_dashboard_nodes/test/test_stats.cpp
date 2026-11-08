#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <nlohmann/json.hpp>

#include "r5_dashboard/stats.hpp"
#include "temp_dir.hpp"

using namespace r5_dashboard::stats;  // NOLINT

TEST(Odometer, CountsDrivingNotJitter)
{
  Odometer odo;
  odo.update(0.0, 0.0);
  for (int i = 0; i < 100; ++i) {  // 1 cm wobble while parked
    odo.update(0.01 * (i % 2), 0.0);
  }
  EXPECT_EQ(odo.distance_m, 0.0);
  for (int i = 1; i <= 10; ++i) {  // drive 1 m in 10 cm steps
    odo.update(0.1 * i, 0.0);
  }
  EXPECT_NEAR(odo.distance_m, 1.0, 1e-9);
}

TEST(Odometer, IgnoresJumpsAndNan)
{
  Odometer odo;
  odo.update(0.0, 0.0);
  odo.update(50.0, 0.0);  // localisation reset
  EXPECT_EQ(odo.distance_m, 0.0);
  odo.update(std::numeric_limits<double>::quiet_NaN(), 0.0);
  odo.update(50.5, 0.0);
  EXPECT_NEAR(odo.distance_m, 0.5, 1e-9);
}

TEST(UsageStats, OperatingTimeSkipsLongGaps)
{
  UsageStats stats;
  stats.add_operating_time(1.0);
  stats.add_operating_time(3600.0);  // suspended
  stats.add_operating_time(-5.0);
  EXPECT_EQ(stats.operating_s, 1.0);
}

TEST(ServiceStatus, Schedule)
{
  UsageStats stats;
  stats.odometer.distance_m = 85000;
  auto s = service_status(stats, 100, 10);
  EXPECT_EQ(s.km_to_service, 15);
  EXPECT_FALSE(s.due_soon);
  EXPECT_FALSE(s.overdue);
  stats.odometer.distance_m = 95000;
  EXPECT_TRUE(service_status(stats, 100, 10).due_soon);
  stats.odometer.distance_m = 101000;
  s = service_status(stats, 100, 10);
  EXPECT_TRUE(s.overdue);
  EXPECT_FALSE(s.due_soon);
  EXPECT_EQ(s.km_to_service, -1);
  stats.last_service = ServiceRecord{0, 101000};
  EXPECT_EQ(service_status(stats, 100, 10).km_to_service, 100);
}

TEST_F(TempDir, StoreRoundTripAndServiceLog)
{
  StatsStore store(dir_);
  EXPECT_EQ(store.load().operating_s, 0);
  UsageStats stats;
  stats.operating_s = 7200;
  stats.odometer.distance_m = 1234.5;
  stats.last_service = ServiceRecord{1000.0, 1000.0};
  store.save(stats);
  const auto loaded = store.load();
  EXPECT_EQ(loaded.operating_s, 7200);
  EXPECT_EQ(loaded.odometer.distance_m, 1234.5);
  ASSERT_TRUE(loaded.last_service);
  EXPECT_EQ(loaded.last_service->time_s, 1000.0);
  EXPECT_EQ(loaded.last_service->distance_m, 1000.0);

  store.log_service({1.0, 2000.0});
  store.log_service({2.0, 4000.0});
  const auto log = nlohmann::json::parse(read("service_log.json"));
  ASSERT_EQ(log.size(), 2u);
  EXPECT_EQ(log[0]["distance_km"], 2.0);
  EXPECT_EQ(log[1]["distance_km"], 4.0);
}

TEST_F(TempDir, ReadsTheFileThePythonNodeWrote)
{
  write("stats.json", "{\n  \"operating_s\": 3600.5,\n  \"distance_m\": 12,\n  \"last_service\": null\n}\n");
  const auto stats = StatsStore(dir_).load();
  EXPECT_EQ(stats.operating_s, 3600.5);
  EXPECT_EQ(stats.odometer.distance_m, 12.0);
  EXPECT_FALSE(stats.last_service);
}

TEST_F(TempDir, DamagedStoreIsMovedAside)
{
  write("stats.json", "{broken");
  StatsStore store(dir_);
  EXPECT_THROW(store.load(), StatsFileDamaged);
  EXPECT_TRUE(std::filesystem::exists(dir_ / "stats.damaged.json"));
  EXPECT_EQ(store.load().operating_s, 0);
}

namespace
{

DiagnosticEntry entry(const std::string & name, int level, const std::string & message = "")
{
  return {name, level, message};
}

}  // namespace

TEST(Alerts, FromDiagnosticsUseLeavesAndKeepStartTime)
{
  AlertTracker t;
  EXPECT_TRUE(
    t.update_diagnostics(
      {entry("/Sensors", kDiagError), entry("/Sensors/IMU", kDiagError, "No data"),
        entry("/Sensors/GPS", kDiagOk)},
      10));
  auto alerts = t.alerts();
  ASSERT_EQ(alerts.size(), 1u);
  EXPECT_EQ(alerts[0].id, "diag:/Sensors/IMU");
  EXPECT_EQ(alerts[0].severity, kError);
  EXPECT_EQ(alerts[0].source, "IMU");
  EXPECT_EQ(alerts[0].message, "No data");
  EXPECT_EQ(alerts[0].since_s, 10);

  // Same state later: nothing changes, start time kept.
  EXPECT_FALSE(t.update_diagnostics({entry("/Sensors/IMU", kDiagError, "No data")}, 20));
  EXPECT_EQ(t.alerts()[0].since_s, 10);

  // Cleared.
  EXPECT_TRUE(t.update_diagnostics({entry("/Sensors/IMU", kDiagOk)}, 30));
  EXPECT_TRUE(t.alerts().empty());
}

TEST(Alerts, StaleIsAWarning)
{
  AlertTracker t;
  t.update_diagnostics({entry("/Motors/Left", kDiagStale)}, 1);
  EXPECT_EQ(t.alerts()[0].severity, kWarning);
  EXPECT_EQ(t.alerts()[0].message, "No recent data");
  t.update_diagnostics({entry("/Motors/Left", kDiagStale, "Timeout")}, 2);
  EXPECT_EQ(t.alerts()[0].message, "No recent data (Timeout)");
  t.update_diagnostics({entry("/Motors/Left", kDiagWarn)}, 3);
  EXPECT_EQ(t.alerts()[0].message, "Problem reported");
}

TEST(Alerts, AcknowledgeAndResetWhenWorse)
{
  AlertTracker t;
  t.update_diagnostics({entry("/GPS", kDiagWarn, "No RTK")}, 1);
  EXPECT_TRUE(t.acknowledge("diag:/GPS"));
  EXPECT_TRUE(t.alerts()[0].acknowledged);
  t.update_diagnostics({entry("/GPS", kDiagWarn, "No RTK")}, 2);
  EXPECT_TRUE(t.alerts()[0].acknowledged);
  t.update_diagnostics({entry("/GPS", kDiagError, "No fix")}, 3);
  EXPECT_FALSE(t.alerts()[0].acknowledged);
  EXPECT_FALSE(t.acknowledge("diag:/nothing"));
}

TEST(Alerts, OwnConditionsSortedBySeverity)
{
  AlertTracker t;
  t.set_condition("robot:info", true, kInfo, "Diagnostics", "x", 1);
  t.set_condition("robot:maint", true, kWarning, "Maintenance", "y", 2);
  t.update_diagnostics({entry("/IMU", kDiagError)}, 3);
  const auto alerts = t.alerts();
  ASSERT_EQ(alerts.size(), 3u);
  EXPECT_EQ(alerts[0].severity, kError);
  EXPECT_EQ(alerts[1].severity, kWarning);
  EXPECT_EQ(alerts[2].severity, kInfo);
  EXPECT_TRUE(t.set_condition("robot:info", false, kInfo, "", "", 4));
  EXPECT_EQ(t.alerts().size(), 2u);
}

TEST(Episode, StartsAfterHoldingAndEndsOnlyWhenClear)
{
  Episode e(10.0);
  e.update(true, false, 0);
  EXPECT_FALSE(e.active());
  e.update(true, false, 9.9);
  EXPECT_FALSE(e.active());
  e.update(true, false, 10.0);
  EXPECT_TRUE(e.active());
  EXPECT_EQ(e.number(), 1);
  e.update(false, false, 11);  // Between the limits: still low.
  EXPECT_TRUE(e.active());
  e.update(false, true, 12);
  EXPECT_FALSE(e.active());
  e.update(true, false, 13);
  e.update(false, false, 14);  // A dip that didn't last.
  e.update(true, false, 20);
  EXPECT_FALSE(e.active());
  EXPECT_EQ(e.number(), 1);
}
