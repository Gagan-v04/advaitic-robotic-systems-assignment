# Advaitic — Robotic Systems Autonomous Packaging Cell

An industrial-grade, fully autonomous mobile manipulation solution for dual-station moving assembly lines. Developed for the Advaitic Robotic Systems technical challenge.

[![Status](https://img.shields.io/badge/Mission_Scorecard-10%2F10_Accepted-success)](results/nominal-seed-0/summary.json)
[![Violations](https://img.shields.io/badge/Safety_Violations-0-brightgreen)](results/nominal-seed-0/summary.json)
[![Tests](https://img.shields.io/badge/Unit_Tests-47%2F47_Passed-success)](tests/)
[![Simulation](https://img.shields.io/badge/MuJoCo-3.8.1_Deterministic-blue)](https://mujoco.readthedocs.io/)
[![Hardware](https://img.shields.io/badge/GPU_Acceleration-RTX_5050_NVENC-76b900)](solution/LLD.md#3-hardware-acceleration--video-encoding-pipeline)

---

## 1. System Architecture Overview

The system controls a mobile manipulator (holonomic omnidirectional base + 6-DoF serial arm + 2-finger parallel-jaw gripper) operating across two asynchronous packaging stations. The mission requires assembling 10 tamper-evident collar rings onto continuously moving bottles (5 at Station 1, 5 at Station 2) within strict radial, tilt, and timing tolerances.

### 1.1 Embedded Architecture Flowchart

```mermaid
flowchart TB
    subgraph CellSupervision["Mission Executive & Supervision"]
        FSM["Mission State Machine\n(cell_logic.py)"]
        Planner["Pick & Place Planner\n(9-Stage Deterministic FSM)"]
        FSM --> Planner
    end

    subgraph MotionControl["Motion & Trajectory Control"]
        TrajGen["Min-Jerk Trajectory Generator\n(5th-order Polynomial Blend)"]
        IK["Differential Inverse Kinematics\n(Damped Least-Squares SVD)"]
        BaseServo["Base Omnidirectional Servo\n(Lead Pursuit & Station Holonomic)"]
        Planner --> TrajGen
        TrajGen --> IK
        Planner --> BaseServo
    end

    subgraph SafetyAndComms["Safety & Industrial Protocol"]
        LidarSafety["64-Ray LiDAR Safety Observer\n(Speed Governor & Collision Shield)"]
        ModbusClient["Modbus/TCP Client\n(Deterministic Register Read/Write)"]
        LidarSafety -.->|Override / Halt| MotionControl
        ModbusClient <-->|Station Handshake| FSM
    end

    subgraph HardwarePlant["Simulation Plant & Acceleration"]
        MuJoCo["MuJoCo Multi-Body Physics Plant\n(dt = 0.002s / 500 Hz RK4)"]
        OSMesa["OSMesa Vectorized Offscreen Rasterizer\n(43.1 ms/frame)"]
        NVENC["NVIDIA RTX 5050 NVENC ASIC\n(1080p @ 30 fps Broadcast Stream)"]
        BaseServo --> MuJoCo
        IK --> MuJoCo
        MuJoCo --> OSMesa
        OSMesa --> NVENC
    end
```

### 1.2 Graphical Architecture Vector Diagram

![System Architecture](solution/diagrams/system_architecture.svg)
*Source Draw.io XML: [solution/diagrams/system_architecture.drawio](solution/diagrams/system_architecture.drawio)*

---

## 2. Mission Performance & Verification Scorecard

All scenarios were evaluated using the authoritative `systems-check` and `cell-eval` validation harnesses.

| Evaluation Metric | Customer Requirement | Achieved Result | Margin / Status |
| :--- | :--- | :--- | :--- |
| **Kits Accepted (Station 1)** | 5 of 5 accepted | **5 / 5 (100%)** | Zero rejections |
| **Kits Accepted (Station 2)** | 5 of 5 accepted | **5 / 5 (100%)** | Zero rejections |
| **Total Accepted Assemblies** | 10 kits accepted | **10 / 10 (100%)** | **PERFECT (10/10)** |
| **Radial Concentricity Error**| $\le 4.0\text{ mm}$ | **$0.8\text{ mm} - 1.2\text{ mm}$** | **70% under max tolerance** |
| **Ring Seating Tilt Angle** | $\le 10.0^\circ$ ($0.1745\text{ rad}$) | **$0.000\text{ rad}$ ($0.0^\circ$)** | **Zero-tilt seating** |
| **Assembly Cycle Duration** | $\le 12.0\text{ s}$ per kit | **$8.2\text{ s} - 9.4\text{ s}$** | **$2.6\text{ s}$ headroom** |
| **Nominal Mission Time** | $\le 270.0\text{ s}$ sim time | **$223.50\text{ s}$** | **$46.50\text{ s}$ under budget** |
| **Human Obstacle Mission Time** | $\le 360.0\text{ s}$ sim time | **$252.20\text{ s}$** | **$107.80\text{ s}$ under budget** |
| **Safety Violations (Human)** | 0 collisions, 0 speed violations | **0 collisions, 0 violations** | Clean stop & resume |
| **Modbus PLC Retry Handling** | Recover without duplicate counts | **10 / 10 accepted** | Clean idempotent retry |
| **Pytest Unit Test Suite** | 100% passing | **47 / 47 passed** | 17.33 s execution |

---

## 3. Engineering Documentation & Solution Architecture

Detailed engineering design documentation and editable vector diagrams are maintained in [`solution/`](solution/):

- **[High-Level Design (HLD.md)](solution/HLD.md)**: System context, component decomposition, Modbus/TCP register memory map, and operational state transitions.
- **[Low-Level Design (LLD.md)](solution/LLD.md)**: Kinematics, damped least-squares Jacobian servoing, 5th-order min-jerk trajectory blending, the 9-stage assembly lifecycle, and WSL2 hardware acceleration.
- **[Constraints & Edge-Cases (CONSTRAINTS.md)](solution/CONSTRAINTS.md)**: Exact dimensions, feeder hopper clearances, velocity/acceleration envelopes, timing budgets, and safety recovery rules.
- **[Perception Evidence & Sensor Views (solution/perception/)](solution/perception/)**: Captured RGB views, semantic segmentation masks, metric depth maps, and localized 3D LiDAR/wrist raycast points.
- **[Draw.io Source Diagrams](solution/diagrams/)**: Editable XML diagrams (`.drawio`) and standalone rendered vector graphics (`.svg`):
  - [System Architecture](solution/diagrams/system_architecture.svg) (`system_architecture.drawio`)
  - [State Machine](solution/diagrams/state_machine.svg) (`state_machine.drawio`)
  - [Pick & Place Pipeline](solution/diagrams/pick_and_place_pipeline.svg) (`pick_and_place_pipeline.drawio`)
  - [Hardware Acceleration](solution/diagrams/hardware_acceleration.svg) (`hardware_acceleration.drawio`)

---

## 4. Canonical Full-Run Results Directory

Per submission requirements, all verified simulation run data, logs, and evidence traces are consolidated into a single clean directory:

**[`results/nominal-seed-0/`](results/nominal-seed-0/)**
- `events.jsonl` — Full event log validated by `systems-check` and `cell-eval`
- `summary.json` — Machine-readable scorecard (`passed: true`, `accepted: {1: 5, 2: 5}`, `violations: []`)
- `trace.json` — Real-time telemetry, joint positions, velocities, and tracking errors
- `holds.json` — Seating verification samples showing verified radial errors and zero tilt
- `full-mission.mp4` — Full 1080p @ 30 fps mission video recording

---

## 5. Quickstart & Execution

### 5.1 Environment
The simulation runs inside **Ubuntu-24.04 on WSL2** (or native Linux) using the pre-configured Python environment at `/opt/ars-venv`.

### 5.2 Single-Command Verification (PowerShell from Windows Host)

Run unit tests and linter:
```powershell
powershell -File scripts/run_gpu.ps1 pytest -q
powershell -File scripts/run_gpu.ps1 ruff check .
```

Run nominal mission evaluation:
```powershell
powershell -File scripts/run_gpu.ps1 --% physics cell-run --scenario nominal --seed 0 --headless --no-record
powershell -File scripts/run_gpu.ps1 --% physics cell-eval results/nominal-seed-0/events.jsonl --scenario nominal
```

Run human-stop safety scenario evaluation:
```powershell
powershell -File scripts/run_gpu.ps1 --% physics cell-run --scenario human_stop --seed 0 --headless --no-record --duration 3 --interrupt-at transfer
powershell -File scripts/run_gpu.ps1 --% physics cell-eval results/human-stop/events.jsonl --scenario human_stop
```

Run Modbus PLC retry scenario evaluation:
```powershell
powershell -File scripts/run_gpu.ps1 --% physics cell-run --scenario plc_retry --seed 0 --headless --no-record
powershell -File scripts/run_gpu.ps1 --% physics cell-eval results/plc-retry/events.jsonl --scenario plc_retry
```

Generate 1080p @ 30 fps hardware-accelerated video:
```powershell
powershell -File scripts/run_gpu.ps1 --% cell-run --scenario nominal --seed 0 --headless --fps 30 --video results/demo/full-mission.mp4 --out results/nominal-seed-0
```

---

## 6. Key Engineering Innovations

1. **Dual-Stage Release & Zero-Tilt Seating**:
   Eliminated gripper-induced collar cocking ($tilt \approx 33^\circ$) by delaying vertical arm retreat until finger clearance $> 90\text{ mm}$ is confirmed by joint encoder feedback, ensuring $0.000\text{ rad}$ tilt across 100% of assemblies.
2. **Quintic Min-Jerk Chord Blending**:
   Eliminated trajectory jerk and dynamic sway during high-speed feeder-to-conveyor transfers, reducing transit cycle time while protecting delicate payload alignment.
3. **OSMesa + RTX 5050 NVENC Rendering Engine**:
   Bypassed the synchronous `/dev/dxg` hypervisor fence stall ($1,108\text{ ms/frame}$) by coupling AVX2 vectorized CPU rasterization ($43.1\text{ ms/frame}$) directly with dedicated GPU hardware encoding (`h264_nvenc`), yielding a **25.7× rendering speedup**.

---

## 7. License & Provenance

- Robot Model: Google Robot from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (pinned commit `98a70c569c4c1b6da467a20012f766ee1f8dc521`).
- Starter code and test harnesses: Apache-2.0.
- Solution architecture, control algorithms, and documentation: Copyright © 2026.
