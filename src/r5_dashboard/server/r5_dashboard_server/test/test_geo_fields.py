import json
import math

import pytest

from r5_dashboard_server.fields import (
    FIX_DGPS,
    FIX_GPS,
    FIX_NONE,
    FIX_RTK_FIXED,
    FIX_RTK_FLOAT,
    FINISHED,
    IDLE,
    RECORDING,
    Field,
    FieldInvalid,
    FieldStore,
    Recorder,
    fix_quality,
    validate,
)
from r5_dashboard_server.geo import (
    LocalProjection,
    close_track,
    point_in_ring,
    ring_area_m2,
    ring_inside,
    ring_self_intersects,
    simplify,
)
from r5_dashboard_server.mapconfig import DEFAULT_CONFIG, MapConfigError, _check, load_map_config, tile_origins

ORIGIN = (-33.86, 151.21)
PROJ = LocalProjection(ORIGIN)


def ll(points_xy):
    """Metres east/north from ORIGIN to lat/lon."""
    return [PROJ.to_latlon(p) for p in points_xy]


SQUARE_100 = ll([(0, 0), (100, 0), (100, 100), (0, 100)])  # 1 ha


# ---- geo ----


def test_projection_round_trip_and_scale():
    lat, lon = PROJ.to_latlon((100.0, 50.0))
    x, y = PROJ.to_xy((lat, lon))
    assert x == pytest.approx(100.0) and y == pytest.approx(50.0)
    # One degree of latitude is about 111 km.
    assert PROJ.to_xy((ORIGIN[0] + 1, ORIGIN[1]))[1] == pytest.approx(111_195, rel=1e-3)


def test_area_self_intersection_and_containment():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert ring_area_m2(square) == 100
    assert not ring_self_intersects(square)
    assert ring_self_intersects([(0, 0), (10, 10), (10, 0), (0, 10)])  # bow tie
    assert point_in_ring((5, 5), square) and not point_in_ring((15, 5), square)
    assert ring_inside([(2, 2), (4, 2), (4, 4)], square)
    assert not ring_inside([(8, 8), (12, 8), (12, 12)], square)


def test_simplify_keeps_shape_within_tolerance():
    line = [(x / 10, 0.02 * math.sin(x)) for x in range(101)]  # 2 cm wiggle along 10 m
    assert simplify(line, 0.1) == [line[0], line[-1]]
    corner = [(0, 0), (5, 0), (10, 0), (10, 5), (10, 10)]
    assert simplify(corner, 0.1) == [(0, 0), (10, 0), (10, 10)]


def test_close_track_drops_repeated_end_and_collinear_points():
    track = [(0, 0), (5, 0), (10, 0), (10, 10), (0, 10), (0, 5), (0, 0)]
    ring = close_track(track, 0.1)
    assert sorted(ring) == sorted([(0, 0), (10, 0), (10, 10), (0, 10)])


# ---- validation ----


def make_field(**kw):
    base = dict(id="", name="East field", boundary=SQUARE_100, headland_width_m=2.0)
    base.update(kw)
    return Field(**base)


def test_valid_field_passes():
    validate(make_field(keep_out=[ll([(40, 40), (60, 40), (60, 60)])]), [])


@pytest.mark.parametrize(
    "changes,reason",
    [
        ({"name": "  "}, "name"),
        ({"name": "x" * 61}, "too long"),
        ({"boundary": SQUARE_100[:2]}, "at least 3"),
        ({"boundary": ll([(0, 0), (100, 100), (100, 0), (0, 100)])}, "crosses itself"),
        ({"keep_out": [ll([(90, 90), (120, 90), (120, 120)])]}, "inside the boundary"),
        ({"keep_out": [ll([(10, 10), (30, 10), (30, 30)]), ll([(20, 20), (40, 20), (40, 40)])]}, "overlap"),
        ({"headland_width_m": 25.0}, "headland"),
        ({"headland_width_m": float("nan")}, "headland"),
        ({"source": "magic"}, "source"),
        ({"boundary": [(95.0, 0.0), (0.0, 1.0), (1.0, 0.0)]}, "valid range"),
    ],
)
def test_invalid_fields(changes, reason):
    with pytest.raises(FieldInvalid, match=reason):
        validate(make_field(**changes), [])


def test_duplicate_name_is_case_insensitive():
    with pytest.raises(FieldInvalid, match="already"):
        validate(make_field(name="east FIELD"), ["East field"])


# ---- storage ----


def test_store_save_list_update_delete(tmp_path):
    clock = iter([1000.0, 1000.0, 2000.0, 2000.0]).__next__
    store = FieldStore(tmp_path, clock=clock)
    saved = store.save(make_field(keep_out=[ll([(40, 40), (60, 40), (60, 60)])]))
    assert saved.id
    assert saved.area_m2 == pytest.approx(10_000, rel=1e-4)
    assert saved.created_s == 1000.0

    # The file is GeoJSON that QGIS can open.
    data = json.loads((tmp_path / "fields" / f"{saved.id}.geojson").read_text())
    assert data["type"] == "FeatureCollection"
    ring = data["features"][0]["geometry"]["coordinates"][0]
    assert ring[0] == ring[-1] and len(ring) == 5
    assert data["features"][0]["properties"]["name"] == "East field"
    assert data["features"][1]["properties"]["role"] == "keep_out"

    saved.name = "East paddock"
    updated = store.save(saved)
    assert updated.created_s == 1000.0 and updated.modified_s == 2000.0
    fields, problems = store.list()
    assert [f.name for f in fields] == ["East paddock"] and problems == []
    assert fields[0].boundary == pytest.approx(SQUARE_100)
    assert len(fields[0].keep_out) == 1

    assert store.delete(saved.id)
    assert not store.delete(saved.id)
    assert store.list() == ([], [])


def test_store_refuses_duplicates_and_unknown_ids(tmp_path):
    store = FieldStore(tmp_path)
    store.save(make_field())
    with pytest.raises(FieldInvalid, match="already"):
        store.save(make_field())
    with pytest.raises(FieldInvalid, match="no longer exists"):
        store.save(make_field(id="0123abcd-0000-0000-0000-000000000000", name="Other"))
    with pytest.raises(FieldInvalid):
        store.delete("../../etc/passwd")


def test_store_skips_unreadable_files(tmp_path):
    store = FieldStore(tmp_path)
    store.save(make_field())
    (tmp_path / "fields" / "broken.geojson").write_text("{nope")
    fields, problems = store.list()
    assert len(fields) == 1 and problems == ["broken.geojson"]


def test_home(tmp_path):
    store = FieldStore(tmp_path)
    assert store.get_home() is None
    store.set_home((-33.86, 151.21))
    assert store.get_home() == (-33.86, 151.21)
    with pytest.raises(FieldInvalid):
        store.set_home((123.0, 0.0))


# ---- recording ----


def test_fix_quality():
    assert fix_quality(-1, 2) == FIX_NONE
    assert fix_quality(0, None) == FIX_GPS
    assert fix_quality(1, None) == FIX_DGPS
    assert fix_quality(2, None) == FIX_RTK_FLOAT
    assert fix_quality(2, 0) == FIX_DGPS
    assert fix_quality(2, 1) == FIX_RTK_FLOAT
    assert fix_quality(2, 2) == FIX_RTK_FIXED


def drive(recorder, xy_points, quality=FIX_RTK_FIXED):
    for p in xy_points:
        recorder.add_fix(PROJ.to_latlon(p), quality)


def square_track(step=0.5):
    side = [i * step for i in range(int(20 / step))]
    return [(x, 0) for x in side] + [(20, y) for y in side] + [(20 - x, 20) for x in side] + [(0, 20 - y) for y in side] + [(0, 0)]


def test_recorder_records_and_simplifies_a_square():
    r = Recorder("rtk_fixed")
    assert r.state == IDLE
    r.start()
    drive(r, square_track())
    assert r.state == RECORDING and len(r.points) > 100
    r.stop()
    assert r.state == FINISHED
    assert len(r.points) == 4
    area = ring_area_m2([PROJ.to_xy(p) for p in r.points])
    assert area == pytest.approx(400, rel=0.01)
    assert r.length_m == pytest.approx(80, rel=0.02)
    assert "4 corners" in r.message


def test_recorder_skips_poor_fixes_and_close_points():
    r = Recorder("rtk_fixed", min_spacing_m=0.2)
    r.start()
    assert not r.add_fix(PROJ.to_latlon((0, 0)), FIX_RTK_FLOAT)
    assert r.skipped == 1
    assert r.add_fix(PROJ.to_latlon((0, 0)), FIX_RTK_FIXED)
    assert not r.add_fix(PROJ.to_latlon((0.1, 0)), FIX_RTK_FIXED)  # too close
    assert r.add_fix(PROJ.to_latlon((0.3, 0)), FIX_RTK_FIXED)
    assert Recorder("any").add_fix(PROJ.to_latlon((0, 0)), FIX_GPS) is False  # not started


def test_recorder_too_short_or_flat():
    r = Recorder("any")
    r.start()
    drive(r, [(0, 0), (1, 0)], FIX_GPS)
    r.stop()
    assert r.state == FINISHED and r.points == [] and "Too few" in r.message
    r.start()
    drive(r, [(x, 0) for x in range(10)], FIX_GPS)
    r.stop()
    assert r.points == [] and "enclose" in r.message


def test_recorder_reports_open_gap_and_clear():
    r = Recorder("any")
    r.start()
    drive(r, [(0, 0), (20, 0), (20, 20), (0, 20), (0, 6)], FIX_GPS)
    r.stop()
    assert "gap was closed" in r.message
    r.clear()
    assert r.state == IDLE and r.points == []


def test_recorder_rejects_unknown_requirement():
    with pytest.raises(ValueError):
        Recorder("perfect")


# ---- map config ----


def test_map_config_default_and_origins(tmp_path):
    assert load_map_config(tmp_path) == DEFAULT_CONFIG
    assert tile_origins(DEFAULT_CONFIG) == ["https://tile.openstreetmap.org"]


def test_map_config_custom_and_bad(tmp_path):
    (tmp_path / "map.json").write_text(
        json.dumps({"layers": [{"name": "Aerial", "tiles": ["https://img.example.org:8443/{z}/{x}/{y}.jpg"]}]})
    )
    cfg = load_map_config(tmp_path)
    assert cfg["layers"][0]["name"] == "Aerial"
    assert tile_origins(cfg) == ["https://img.example.org:8443"]
    with pytest.raises(MapConfigError):
        _check({"layers": [{"tiles": ["http://plain.example.org/{z}/{x}/{y}"]}]})
    (tmp_path / "map.json").write_text("{bad json")
    assert load_map_config(tmp_path) == DEFAULT_CONFIG
