"""Pure-Python tests for frames, the feeder, navigation and the PLC adapter."""

import math
import threading

import numpy as np
import pytest

from advaitic_cell.feeder import Feeder
from advaitic_cell.frames import (
    bottle_local_x,
    bottle_world,
    inspection_delay,
    kit_world,
    local_to_world,
    ring_world,
    world_to_local,
)
from advaitic_cell.navigate import (
    DiffDriveTracker,
    integrate_unicycle,
    obstacle_clear,
    plan_to_dock,
)
from advaitic_cell.plc_adapter import RobotPLC, command_frame
from advaitic_cell.spec import ENTRY_X, INSPECT_X, PITCH_M, dock_pose, scenario_params, start_pose
from advaitic_systems.modbus import PLCServer


def test_station_b_offsets_rotate_with_the_belt():
    point = local_to_world(1, 0.2, 0.045, 0.9)
    assert point == pytest.approx((3.2, 1.245, 0.9))
    back = world_to_local(1, point[0], point[1])
    assert back == pytest.approx((0.2, 0.045))
    rotated = local_to_world(2, 0.2, 0.045, 0.9)
    assert rotated == pytest.approx((2.8, 4.755, 0.9))
    assert world_to_local(2, rotated[0], rotated[1]) == pytest.approx((0.2, 0.045))


def test_ring_is_ahead_of_the_bottle_on_both_stations():
    for station in (1, 2):
        bottle = bottle_world(station, -0.30)
        ring = ring_world(station, -0.30)
        # 140 mm ahead along station +x, same lateral offset.
        local_b = world_to_local(station, bottle[0], bottle[1])
        local_r = world_to_local(station, ring[0], ring[1])
        assert local_r[0] - local_b[0] == pytest.approx(0.14)
        assert local_r[1] == pytest.approx(local_b[1])
        kit_x, kit_y, kit_z, _yaw = kit_world(station, -0.30)
        # Kit top is the frame origin, so z is the customer kit-top height.
        assert kit_z == pytest.approx(0.775)
        assert world_to_local(station, kit_x, kit_y)[0] == pytest.approx(-0.30 + 0.07)


def test_inspection_delay_matches_the_checker():
    assert inspection_delay(0.03) == pytest.approx(0.85 / 0.03)
    assert INSPECT_X - ENTRY_X == pytest.approx(0.85)
    assert bottle_local_x(10.0, 10.0 + 0.85 / 0.03, 0.03) == pytest.approx(INSPECT_X)


def test_scenario_seeds_are_stable_and_inside_the_customer_range():
    first = scenario_params("nominal", 0)
    assert first == scenario_params("nominal", 0)
    assert first != scenario_params("nominal", 1)
    for seed in range(6):
        params = scenario_params("nominal", seed)
        assert 0.025 <= params.belt_speed_mps <= 0.035
        assert 0.0 <= params.release_phase_s < 1.0
    with pytest.raises(ValueError):
        scenario_params("nominal", 0, human_duration=1.0)


def test_feeder_respects_phase_pitch_and_the_five_kit_cap():
    feeder = Feeder(1, 0.03, 0.5, 101)
    released = []
    t = 0.0
    accepted = 0
    while t < 200 and len(released) < 5:
        item = feeder.try_release(t, True, accepted)
        if item:
            released.append((t, item))
        # Inspect a kit 0.85/speed after its arrival so slots free the way the line does.
        for _spawn_t, (kit_id, arrival) in list(released):
            if kit_id in feeder.in_flight and t >= arrival + 0.85 / 0.03:
                feeder.mark_inspected(kit_id)
                accepted += 1
        t = round(t + 0.05, 4)
    arrivals = [item[1][1] for item in released]
    assert [item[1][0] for item in released] == [101, 102, 103, 104, 105]
    assert arrivals[0] == pytest.approx(0.5)
    gaps = np.diff(arrivals)
    assert gaps == pytest.approx([PITCH_M / 0.03] * 4)
    # A sixth kit cannot be released while five are still unresolved.
    blocked = Feeder(1, 0.03, 0.0, 1)
    ids = []
    for step in range(2000):
        item = blocked.try_release(step * 0.05, True, accepted=0)
        if item:
            ids.append(item[0])
    assert len(ids) == 5


def test_replacement_keeps_pitch_after_a_reject():
    feeder = Feeder(2, 0.025, 0.0, 201)
    first = None
    t = 0.0
    while first is None:
        first = feeder.try_release(t, True, 0)
        t += 0.01
    feeder.mark_inspected(first[0])
    second = None
    while second is None and t < 30:
        second = feeder.try_release(t, True, accepted=0)
        t += 0.01
    assert second[1] - first[1] >= PITCH_M / 0.025 - 1e-6


def test_docks_and_aisle_are_navigable():
    start = start_pose()
    for station in (1, 2):
        dock = dock_pose(station)
        assert obstacle_clear(dock[0], dock[1])
        path, dock_xy, yaw = plan_to_dock((start[0], start[1]), station)
        assert all(obstacle_clear(float(p[0]), float(p[1])) for p in path)
        tracker = DiffDriveTracker(path, dock_xy, yaw)
        x, y, heading = start
        for _ in range(2500):
            speed, yaw_rate = tracker.command(x, y, heading)
            assert abs(speed) <= 0.35 + 1e-9
            assert abs(yaw_rate) <= 0.55
            x, y, heading = integrate_unicycle(x, y, heading, speed, yaw_rate, 0.05)
            assert obstacle_clear(x, y, radius=0.20)
            if tracker.done:
                break
        assert tracker.done
        assert np.hypot(x - dock[0], y - dock[1]) < 0.02
        assert abs((heading - yaw + np.pi) % (2 * np.pi) - np.pi) < 0.03
        start = (x, y, heading)


def test_real_socket_ack_drop_does_not_double_apply():
    with PLCServer(("127.0.0.1", 0)) as server:
        server.model.arrive(1, 42)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            port = server.server_address[1]
            robot = RobotPLC(port=port)
            ready = robot.command(1, 0, 1)
            assert ready.sequence == 1 and not ready.duplicated
            dropped = robot.command(1, 42, 3, drop_ack=True)
            assert dropped.sequence == 2
            assert dropped.duplicated
            assert dropped.first_hex is not None and len(dropped.first_hex) == 42
            assert command_frame(1, 2, 1, 42, 3)[7:] == bytes.fromhex(dropped.first_hex)[7:]
            assert bytes.fromhex(dropped.first_hex)[7:] == bytes.fromhex(dropped.retry_hex)[7:]
            assert dropped.accepted_before == dropped.accepted_after == 0
            assert server.model.last_status == 2
            assert len(server.model.commands) == 2  # READY + one ASSEMBLY_DONE
            assert server.model.accepted[1] == 0
        finally:
            server.shutdown()
            thread.join()


def test_minjerk_is_smooth_and_hits_the_ends():
    from advaitic_cell.motion import CartesianServo, minjerk

    assert minjerk(0.0) == pytest.approx(0.0)
    assert minjerk(1.0) == pytest.approx(1.0)
    mid = [minjerk(t / 50) for t in range(51)]
    diffs = np.diff(mid)
    assert all(d >= -1e-12 for d in diffs)
    servo = CartesianServo()
    start = np.array([0.0, 0.0, 1.0])
    goal = np.array([0.20, 0.0, 1.0])
    servo.reset(start)
    speeds = []
    pos = start.copy()
    for _ in range(400):
        nxt = servo.step(goal, 0.004, vertical=False)
        speeds.append(float(np.linalg.norm(nxt - pos) / 0.004))
        pos = nxt
    assert speeds[0] < 0.08
    assert max(speeds) < 0.40
    assert float(np.linalg.norm(pos - goal)) < 0.01


def test_vertical_segment_does_not_cut_the_corner():
    from advaitic_cell.motion import CartesianServo

    servo = CartesianServo()
    start = np.array([1.0, 2.0, 1.00])
    servo.reset(start)
    goal = np.array([1.04, 2.0, 0.78])
    xs = []
    for _ in range(500):
        pos = servo.step(goal, 0.004, vertical=True)
        xs.append(float(pos[0]))
    # XY may track the 4 cm belt motion, but z is a committed min-jerk, not a
    # diagonal jump through the plate.
    assert min(xs) >= 0.999
    assert abs(float(servo.pos[2]) - 0.78) < 0.008
    assert servo.vertical_done()


def test_lateral_move_holds_height_until_xy_is_close():
    from advaitic_cell.motion import CartesianServo

    servo = CartesianServo()
    servo.reset(np.array([0.0, 0.0, 1.20]))
    goal = np.array([0.40, 0.0, 0.86])
    for _ in range(80):
        pos = servo.step(goal, 0.004, vertical=False, z_floor=0.91)
        xy = math.hypot(float(pos[0] - goal[0]), float(pos[1] - goal[1]))
        if xy > 0.12:
            assert float(pos[2]) >= 0.905, f"z={pos[2]:.3f} while xy={xy:.3f}"
            assert float(pos[2]) > 0.86 + 0.02
