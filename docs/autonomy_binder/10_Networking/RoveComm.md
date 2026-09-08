# RoveComm Networking Protocol

`RoveComm` is a custom in-house application-layer communication protocol developed by the Mars Rover Design Team. It connects the Autonomy Software (executing on the Jetson computer), distributed embedded microcontrollers (Drive Board, Navigation Board, Multimedia Board), and the Basestation Command and Control (C2) console.

The protocol is included as a git submodule in `external/rovecomm`.

---

## 1. Transport Protocols: UDP vs TCP

RoveComm provides dual transport layers tailored to different telemetry requirements:

### A. RoveComm UDP (User Datagram Protocol)
- **Primary Use**: High-rate, periodic, loss-tolerant sensor and actuator telemetry.
- **Examples**:
  - `manifest::Core::COMMANDS["DRIVELEFTRIGHT"]` transmitted at 60 Hz to the motor controller.
  - `manifest::Nav::TELEMETRY["GPSLATLON"]` and `["IMUDATA"]` streaming from the Navigation Board.
  - Periodic lighting commands (`STATEDISPLAY`, `LEDRGB`).
  - Log message streaming to the Basestation console.
- **Behavior**: Socket transmission is non-blocking. If a wireless frame is dropped, subsequent packets overwrite the dropped data without retransmission delays.
- **Binding**: Handled by `network::g_pRoveCommUDPNode` on port `manifest::General::ETHERNET_UDP_PORT` (default 11000).

### B. RoveComm TCP (Transmission Control Protocol)
- **Primary Use**: Low-rate, mission-critical, guaranteed-delivery commands.
- **Examples**:
  - Mission leg injections (`ADDPOSITIONLEG`, `ADDMARKERLEG`, `ADDOBJECTLEG`).
  - Queue clearing commands (`CLEARWAYPOINTS`).
  - Runtime logging level reconfigurations (`SETLOGGINGLEVELS`).
- **Behavior**: Uses stream-based delivery with kernel-level acknowledgments and ordered sequencing.
- **Binding**: Handled by `network::g_pRoveCommTCPNode` bound to `constants::ROVECOMM_TCP_INTERFACE_IP` and `manifest::General::ETHERNET_TCP_PORT` (default 11000).

---

## 2. The RoveComm Manifest

To maintain compatibility between C++ embedded firmware, C++ autonomy software, and Python base station GUI software, all message definitions are centralized in `RoveCommManifest.h` (generated from `manifest.json`).

Each manifest entry defines three fields:
1. **`DATA_ID`**: A unique 16-bit unsigned integer identifier.
2. **`DATA_COUNT`**: Expected number of array elements in the payload.
3. **`DATA_TYPE`**: Primitive data type identifier:
   - `UINT8_T`, `INT8_T`
   - `UINT16_T`, `INT16_T`
   - `UINT32_T`, `INT32_T`
   - `FLOAT_T` (32-bit IEEE 754)
   - `DOUBLE_T` (64-bit IEEE 754)
   - `CHAR_T`

---

## 3. Packet Structure: `RoveCommPacket<T>`

Network payloads are encapsulated within the templated `RoveCommPacket<T>` struct:

```cpp
template<typename T>
struct RoveCommPacket
{
    uint16_t unDataId;            // Message ID from manifest
    uint16_t unDataCount;         // Array element count
    manifest::DataTypes eDataType; // Data type enum
    std::vector<T> vData;         // Payload vector
};
```

When transmitted, network byte order conversions (`htonll`, `ntohll`) ensure consistent endianness across x86_64 host machines and ARM64 Jetson architectures.

---

## 4. Sending Telemetry and Commands

To transmit a packet, instantiate `RoveCommPacket<T>`, set the manifest parameters, populate `vData`, and dispatch via the node pointer:

```cpp
// Example: Sending motor powers over UDP (from DriveBoard.cpp)
rovecomm::RoveCommPacket<float> stPacket;
stPacket.unDataId    = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID;
stPacket.unDataCount = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT;
stPacket.eDataType   = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE;

stPacket.vData.emplace_back(fLeftTrackPower);
stPacket.vData.emplace_back(fRightTrackPower);

// Transmit to the Core board IP
network::g_pRoveCommUDPNode->SendUDPPacket(
    stPacket,
    "192.168.1.130",
    constants::ROVECOMM_OUTGOING_UDP_PORT
);
```

---

## 5. Asynchronous Callbacks

Incoming messages are processed asynchronously using callback handlers registered with the UDP or TCP node:

```cpp
// 1. Define callback lambda
const std::function<void(const rovecomm::RoveCommPacket<double>&, const sockaddr_in&)> AddPositionLegCallback =
    [this](const rovecomm::RoveCommPacket<double>& stPacket, const sockaddr_in& stdAddr)
{
    double dLat = stPacket.vData[0];
    double dLon = stPacket.vData[1];
    int nLegID  = static_cast<int>(stPacket.vData[2]);

    this->AddWaypoint(geoops::GPSCoordinate(dLat, dLon), geoops::WaypointType::eNavigationWaypoint, 0.0, nLegID);
};

// 2. Register callback on the UDP node
network::g_pRoveCommUDPNode->AddUDPCallback<double>(
    AddPositionLegCallback,
    manifest::Autonomy::COMMANDS.find("ADDPOSITIONLEG")->second.DATA_ID
);
```

Because `RoveCommUDP` runs on its own background thread, incoming socket packets are unpacked, matched to their `DATA_ID`, and dispatched to their registered callbacks automatically without polling.
