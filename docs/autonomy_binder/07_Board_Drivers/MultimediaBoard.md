# Multimedia Board Driver

The `MultimediaBoard` driver (`src/drivers/MultimediaBoard.h` & `MultimediaBoard.cpp`) manages the rover's visual signaling hardware, controlling high-intensity LED light strips and indicators to communicate operational status to judges and operators.

---

## 1. Primary Responsibilities

1. **State-Driven Lighting**: Translates high-level autonomy states into competition-compliant LED colors.
2. **Dual Telemetry and Command Transmission**: Dispatches both a Basestation telemetry packet (`manifest::Autonomy::TELEMETRY["STATEDISPLAY"]`) and a hardware microcontroller command packet (`manifest::Core::COMMANDS["STATEDISPLAY"]`).
3. **Direct RGB Control**: Provides low-level interfaces (`SendRGB()`) to command custom hexadecimal or RGB values directly.

---

## 2. Operational Lighting States

Lighting behavior is governed by the `MultimediaBoardLightingState` enumeration:

| Enum State | Commanded Color | Associated Robot Status |
| :--- | :--- | :--- |
| `eOff` | Black / Off `[0, 0, 0]` | System shutdown, idle standby, or unpowered LEDs. |
| `eAutonomy` | **Solid Red** | Autonomy state machine active and in control of chassis movement. |
| `eTeleOp` | **Solid Blue** | Manual teleoperation active; operator joystick override. |
| `eReachedGoal` | **Flashing Green** | Target waypoint reached, ArUco post verified, or prop detected. |
| `eCustom` | User-defined RGB | Diagnostic test patterns or custom animations. |

---

## 3. Network Transmission Protocol

When `SendLightingState()` is called:
1. `stTelemPacket` is constructed with Data ID `manifest::Autonomy::TELEMETRY["STATEDISPLAY"]`, notifying the Basestation GUI to update on-screen indicators.
2. `stCorePacket` is constructed with Data ID `manifest::Core::COMMANDS["STATEDISPLAY"]`, instructing the physical Core microcontroller to toggle LED driver relays or WS2812B strips.
3. Packets are transmitted over UDP via `network::g_pRoveCommUDPNode`. Because lighting commands represent discrete state changes rather than continuous control loops, transmission occurs only on state transitions to conserve network bandwidth.

---

## 4. Public Interface Summary

```cpp
enum class MultimediaBoardLightingState
{
    eOff,
    eTeleOp,
    eAutonomy,
    eReachedGoal,
    eCustom
};

void SendLightingState(MultimediaBoardLightingState eState);
void SendRGB(const RGB& stRGB);
MultimediaBoardLightingState GetCurrentLightingState() const;
```
