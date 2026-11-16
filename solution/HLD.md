# High-Level Design (HLD): Autonomous Mobile Manipulator Assembly Cell

## 1. Executive Summary

The Advaitic Robotic Systems manufacturing cell is a fully autonomous dual-station packaging assembly environment. A mobile manipulator comprising a differential-drive mobile base, a 6-DoF serial arm, and a 2-finger parallel-jaw gripper must navigate between two distinct workcells, coordinate with station PLCs via Modbus/TCP, and seat 10 precision tamper-evident collar rings onto continuously moving bottles across two conveyor belts (5 rings at Station 1, 5 rings at Station 2).

All operations must strictly satisfy sub-millimeter tolerances, physical clearance limits, dynamic timing budgets, safety supervisor standards, and Modbus/TCP communication protocols without any manual intervention.

---

## 2. System Architecture

The software architecture is decoupled into three primary execution tiers:
1. **Decision & Control Tier (Top)**: Mission Executive finite-state machine (FSM), Cartesian servoing, and min-jerk trajectory generators.
2. **Safety & Communications Tier (Middle)**: Real-time 64-ray LiDAR obstacle observer and industrial Modbus/TCP client interface.
3. **Physics & Actuation Tier (Bottom)**: 500 Hz MuJoCo multi-body dynamic simulation engine, offscreen sensor pipeline, and hardware-accelerated video rendering.

### 2.1 Embedded Architecture Diagram (Interactive Flowchart)

```mermaid
flowchart TB
    subgraph CellSupervision["T1: Mission Executive & State Supervision"]
        FSM["Mission State Machine\n(cell_logic.py)"]
        Planner["Pick & Place Planner\n(9-Stage Deterministic FSM)"]
        FSM --> Planner
    end

    subgraph MotionControl["T2: Motion & Trajectory Generation"]
        TrajGen["Min-Jerk Trajectory Generator\n(5th-order Polynomial Blend)"]
        IK["Differential Inverse Kinematics\n(Damped Least-Squares SVD)"]
        BaseServo["Diff-Drive Navigation & Alignment Tracker\n(Pure Pursuit & Dock Creep)"]
        Planner --> TrajGen
        TrajGen --> IK
        Planner --> BaseServo
    end

    subgraph SafetyAndComms["T3: Safety & Industrial Modbus/TCP Layer"]
        LidarSafety["64-Ray LiDAR Safety Observer\n(Speed Governor & Collision Shield)"]
        ModbusClient["Modbus/TCP Client\n(Deterministic Register Read/Write)"]
        LidarSafety -.->|Override / Halt| MotionControl
        ModbusClient <-->|Station Handshake| FSM
    end

    subgraph HardwarePlant["T4: Physics Plant & Acceleration Engine"]
        MuJoCo["MuJoCo Multi-Body Physics Plant\n(dt = 0.002s / 500 Hz RK4)"]
        OSMesa["OSMesa Vectorized Offscreen Rasterizer\n(43.1 ms/frame)"]
        NVENC["NVIDIA RTX 5050 NVENC ASIC\n(1080p @ 30 fps Broadcast Stream)"]
        BaseServo --> MuJoCo
        IK --> MuJoCo
        MuJoCo --> OSMesa
        OSMesa --> NVENC
    end
```

### 2.2 Graphical Architecture Vector Diagram

![System Architecture](diagrams/system_architecture.svg)
*Source Draw.io XML: [system_architecture.drawio](diagrams/system_architecture.drawio)*

---

## 3. Operational State Machine

The top-level operational flow coordinates navigation between stations and the kit assembly cycles:

```mermaid
stateDiagram-v2
    [*] --> CellInit: Initialize Sensors & Modbus Handshake
    CellInit --> NavStation1: Start Mission
    NavStation1 --> WorkStation1: Base Arrived at S1 (1.60, 0.45)
    
    state WorkStation1 {
        [*] --> S1_AssembleKit
        S1_AssembleKit --> S1_VerifyAccept: Drop Ring & Settle
        S1_VerifyAccept --> S1_AssembleKit: Next Kit (1..5)
        S1_VerifyAccept --> S1_Done: kits_accepted == 5
    }

    WorkStation1 --> NavStation2: S1 Batch Complete
    NavStation2 --> WorkStation2: Base Arrived at S2 (3.20, 0.45)

    state WorkStation2 {
        [*] --> S2_AssembleKit
        S2_AssembleKit --> S2_VerifyAccept: Drop Ring & Settle
        S2_VerifyAccept --> S2_AssembleKit: Next Kit (1..5)
        S2_VerifyAccept --> S2_Done: kits_accepted == 5
    }

    WorkStation2 --> MissionComplete: S2 Batch Complete (Total: 10/10)
    MissionComplete --> [*]

    state SafetyInterlock {
        SafetyStop: Human Intrusion Detected (d < 1.2m)
        Resume: Path Clear (d > 1.5m for 500ms)
    }

    NavStation1 --> SafetyStop: Human enters safety zone
    WorkStation1 --> SafetyStop: Human enters safety zone
    NavStation2 --> SafetyStop: Human enters safety zone
    WorkStation2 --> SafetyStop: Human enters safety zone
    SafetyStop --> Resume: Safe clearance confirmed
```

### 3.1 Graphical State Machine Vector Diagram

![State Machine](diagrams/state_machine.svg)
*Source Draw.io XML: [state_machine.drawio](diagrams/state_machine.drawio)*

---

## 4. Communication & Safety Architecture

### 4.1 Modbus/TCP Industrial Protocol
Communication with the physical plant's Programmable Logic Controllers (PLCs) follows standard Modbus/TCP register mappings over port 502:

| Register Address | Name | Type | Direction | Description |
| :--- | :--- | :--- | :--- | :--- |
| `30001` | Station Status | Holding Register | In (Read) | Bitmask: Station ready, belt active, part present |
| `30002` | Kit Accepted Count | Holding Register | In (Read) | Accumulated accepted kit count (0–5 per station) |
| `40001` | Watchdog Heartbeat | Holding Register | Out (Write) | Incrementing counter verifying robot controller alive |
| `40002` | Robot Stage Status | Holding Register | Out (Write) | Current assembly phase (0=Nav, 1=Pick, 2=Track, 3=Seat) |
| `40003` | Emergency Stop ACK | Holding Register | Out (Write) | Safety supervisor confirmation of active halt |

### 4.2 LiDAR Safety & Dynamic Speed Governor
- **Sensor Setup**: 64-ray radial planar LiDAR array centered on the mobile base at $z = 0.25\text{ m}$.
- **Warning Zone ($d < 1.8\text{ m}$)**: Linear velocity scaled down to 40% using smooth exponential decay.
- **Stop Zone ($d < 1.2\text{ m}$)**: Actuator command immediately forced to zero; mechanical brakes clamped; trajectory interpolator frozen in place.
- **Hysteresis Clearing**: Resumes motion only when the detected human boundary exceeds $1.5\text{ m}$ continuously for $\ge 500\text{ ms}$, ensuring zero jerk spikes or oscillating stops.
