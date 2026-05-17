# RoveComm Networking Protocol

`RoveComm` is a custom, in-house communication protocol developed by the Mars Rover Design Team. It acts as the "nervous system" of the rover, facilitating all communication between the Autonomy Software (running on the Jetson/Main Computer), the hardware microcontrollers (Drive Board, Navigation Board, Multimedia Board), and the Basestation operators.

It is included in this repository as an external git submodule (`external/rovecomm`).

## Architecture & Transport Layers

RoveComm supports two underlying transport protocols depending on the needs of the specific data stream:

### 1. RoveComm UDP (User Datagram Protocol)
- **Use Case**: High-frequency, continuous, loss-tolerant data.
- **Examples**: Sending `DRIVELEFTRIGHT` motor commands at 60Hz, or receiving continuous GPS and IMU telemetry from the Navigation Board.
- **Why**: If a UDP packet drops over the wireless network, the system doesn't waste time trying to retransmit it. It just waits for the next packet 16 milliseconds later.
- **Initialization**: `network::g_pRoveCommUDPNode->InitUDPSocket(...)`

### 2. RoveComm TCP (Transmission Control Protocol)
- **Use Case**: Critical, one-time, guaranteed-delivery data.
- **Examples**: Sending a new mission Waypoint list from the basestation, or sending an absolute State Machine override command.
- **Why**: Ensures that critical configuration data arrives reliably and in the correct order.
- **Initialization**: `network::g_pRoveCommTCPNode->InitTCPSocket(...)`

## The RoveComm Manifest

To ensure that the C++ Autonomy Software, the Python Basestation, and the C++ Microcontrollers all agree on *what* a piece of data means, the protocol uses the **RoveComm Manifest**.

The manifest (`RoveCommManifest.h` / `manifest.json`) defines every possible packet type in the system. It maps a human-readable string (like `"DRIVELEFTRIGHT"`) to:
1. **Data ID (`unDataId`)**: A unique 16-bit integer identifying the command.
2. **Data Count (`unDataCount`)**: How many elements are expected in the array (e.g., 2 for Left and Right powers).
3. **Data Type (`eDataType`)**: The type of the elements (e.g., `FLOAT_T`, `INT32_T`).

*Note: The `RoveCommManifest.h` file is auto-generated. Do not edit it manually. Instead, edit the upstream `manifest.json`.*

## Packet Structure

Data is packaged into the templated `RoveCommPacket<T>` struct before being sent over the network:

```cpp
template<typename T>
struct RoveCommPacket
{
    uint16_t unDataId;            // From Manifest
    uint16_t unDataCount;         // From Manifest
    manifest::DataTypes eDataType; // From Manifest
    std::vector<T> vData;         // The actual payload array
};
```
When transmitted, `RoveComm` uses specific macros (`htonll`, `ntohll`) to handle endianness conversions, ensuring that data is packed correctly regardless of the CPU architecture (ARM vs x86).

## Sending Data

To send data, you populate a `RoveCommPacket` struct and call the `SendUDPPacket` or `SendTCPPacket` method.

Example from `DriveBoard.cpp`:
```cpp
rovecomm::RoveCommPacket<float> stPacket;
stPacket.unDataId    = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID;
stPacket.unDataCount = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_COUNT;
stPacket.eDataType   = manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_TYPE;

// Attach the payload
stPacket.vData.emplace_back(leftDrivePower);
stPacket.vData.emplace_back(rightDrivePower);

// Send over the network
network::g_pRoveCommUDPNode->SendUDPPacket(stPacket, "192.168.1.130", constants::ROVECOMM_OUTGOING_UDP_PORT);
```

## Receiving Data via Callbacks

Instead of writing a massive, blocking `switch` statement to handle incoming data, `RoveComm` uses an asynchronous Callback architecture.

When the autonomy software boots up (in `main.cpp` or inside driver constructors), it registers callback functions tied to specific Data IDs.

Example from `DriveBoard.h`:
```cpp
// 1. Define the callback lambda
const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> SetMaxSpeedCallback =
    [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
{
    // Handle the incoming speed multiplier
    float multiplier = stPacket.vData[0];
    this->SetMaxDriveEffort(multiplier);
};

// 2. Register it with the UDP Node
network::g_pRoveCommUDPNode->AddUDPCallback<float>(
    SetMaxSpeedCallback,
    manifest::Core::COMMANDS.find("SETMAXSPEED")->second.DATA_ID
);
```

Because `RoveCommUDP` inherits from `AutonomyThread`, it runs in the background. When a UDP packet hits the socket, the thread decodes the packet header, checks if the `unDataId` matches any registered callbacks, and executes the attached lambda function automatically.

## Pub/Sub Architecture

To prevent microcontrollers from spamming the network with telemetry data when no one is listening, the `RoveComm` protocol supports a rudimentary Publish/Subscribe model.

When Autonomy starts up, the `NavigationBoard` driver sends a specific `SUBSCRIBE_DATA_ID` packet to the NavBoard. Only then does the NavBoard begin streaming its `GPSLATLON` packets back to the Jetson's IP address.
