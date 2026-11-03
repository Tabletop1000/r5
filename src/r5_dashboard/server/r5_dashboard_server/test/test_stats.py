import json

from r5_dashboard_server.stats import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_STALE,
    DIAG_WARN,
    ERROR,
    INFO,
    WARNING,
    AlertTracker,
    DiagnosticEntry,
    Odometer,
    ServiceRecord,
    StatsFileDamaged,
    StatsStore,
    UsageStats,
    service_status,
)


def test_odometer_counts_driving_not_jitter():
    odo = Odometer()
    odo.update(0.0, 0.0)
    for i in range(100):  # 1 cm wobble while parked
        odo.update(0.01 * (i % 2), 0.0)
    assert odo.distance_m == 0.0
    for i in range(1, 11):  # drive 1 m in 10 cm steps
        odo.update(0.1 * i, 0.0)
    assert abs(odo.distance_m - 1.0) < 1e-9


def test_odometer_ignores_jumps_and_nan():
    odo = Odometer()
    odo.update(0.0, 0.0)
    odo.update(50.0, 0.0)  # localisation reset
    assert odo.distance_m == 0.0
    odo.update(float("nan"), 0.0)
    odo.update(50.5, 0.0)
    assert abs(odo.distance_m - 0.5) < 1e-9


def test_operating_time_skips_long_gaps():
    stats = UsageStats()
    stats.add_operating_time(1.0)
    stats.add_operating_time(3600.0)  # suspended
    stats.add_operating_time(-5.0)
    assert stats.operating_s == 1.0


def test_service_schedule():
    stats = UsageStats()
    stats.odometer.distance_m = 85_000
    s = service_status(stats, 100, 10)
    assert s.km_to_service == 15 and not s.due_soon and not s.overdue
    stats.odometer.distance_m = 95_000
    assert service_status(stats, 100, 10).due_soon
    stats.odometer.distance_m = 101_000
    s = service_status(stats, 100, 10)
    assert s.overdue and not s.due_soon and s.km_to_service == -1
    stats.last_service = ServiceRecord(time_s=0, distance_m=101_000)
    assert service_status(stats, 100, 10).km_to_service == 100


def test_store_round_trip_and_service_log(tmp_path):
    store = StatsStore(tmp_path)
    assert store.load().operating_s == 0
    stats = UsageStats(operating_s=7200)
    stats.odometer.distance_m = 1234.5
    stats.last_service = ServiceRecord(1000.0, 1000.0)
    store.save(stats)
    loaded = store.load()
    assert loaded.operating_s == 7200
    assert loaded.odometer.distance_m == 1234.5
    assert loaded.last_service == ServiceRecord(1000.0, 1000.0)

    store.log_service(ServiceRecord(1.0, 2000.0))
    store.log_service(ServiceRecord(2.0, 4000.0))
    log = json.loads((tmp_path / "service_log.json").read_text())
    assert [e["distance_km"] for e in log] == [2.0, 4.0]


def test_damaged_store_is_moved_aside(tmp_path):
    (tmp_path / "stats.json").write_text("{broken")
    store = StatsStore(tmp_path)
    try:
        store.load()
        raise AssertionError("expected StatsFileDamaged")
    except StatsFileDamaged as exc:
        assert exc.backup.exists()
    assert store.load().operating_s == 0


def entry(name, level, message=""):
    return DiagnosticEntry(name, level, message)


def test_alerts_from_diagnostics_use_leaves_and_keep_start_time():
    t = AlertTracker()
    assert t.update_diagnostics(
        [entry("/Sensors", DIAG_ERROR), entry("/Sensors/IMU", DIAG_ERROR, "No data"), entry("/Sensors/GPS", DIAG_OK)],
        now_s=10,
    )
    [a] = t.alerts
    assert (a.id, a.severity, a.source, a.message, a.since_s) == ("diag:/Sensors/IMU", ERROR, "IMU", "No data", 10)

    # Same state later: nothing changes, start time kept.
    assert not t.update_diagnostics([entry("/Sensors/IMU", DIAG_ERROR, "No data")], now_s=20)
    assert t.alerts[0].since_s == 10

    # Cleared.
    assert t.update_diagnostics([entry("/Sensors/IMU", DIAG_OK)], now_s=30)
    assert t.alerts == []


def test_stale_is_a_warning():
    t = AlertTracker()
    t.update_diagnostics([entry("/Motors/Left", DIAG_STALE)], now_s=1)
    assert t.alerts[0].severity == WARNING
    assert "No recent data" in t.alerts[0].message


def test_acknowledge_and_reset_when_worse():
    t = AlertTracker()
    t.update_diagnostics([entry("/GPS", DIAG_WARN, "No RTK")], now_s=1)
    assert t.acknowledge("diag:/GPS")
    assert t.alerts[0].acknowledged
    t.update_diagnostics([entry("/GPS", DIAG_WARN, "No RTK")], now_s=2)
    assert t.alerts[0].acknowledged
    t.update_diagnostics([entry("/GPS", DIAG_ERROR, "No fix")], now_s=3)
    assert not t.alerts[0].acknowledged
    assert not t.acknowledge("diag:/nothing")


def test_own_conditions_sorted_by_severity():
    t = AlertTracker()
    t.set_condition("robot:info", True, INFO, "Diagnostics", "x", 1)
    t.set_condition("robot:maint", True, WARNING, "Maintenance", "y", 2)
    t.update_diagnostics([entry("/IMU", DIAG_ERROR)], now_s=3)
    assert [a.severity for a in t.alerts] == [ERROR, WARNING, INFO]
    assert t.set_condition("robot:info", False, INFO, "", "", 4)
    assert len(t.alerts) == 2
