# Navigation Board Driver

The `NavigationBoard` driver (`src/drivers/NavigationBoard.h` & `NavigationBoard.cpp`) interfaces with the rover's GPS receivers, RTK systems, and IMU compass to maintain ground-truth positioning.

---

## 1. Primary Responsibilities

1. **Telemetry Ingestion**: Subscribes to high-frequency RoveComm UDP streams from the physical Navigation Board microcontroller.
2. **Geodetic Projections**: Automatically converts raw WGS84 GPS latitude/longitude/altitude coordinates into Universal Transverse Mercator (UTM) Cartesian coordinates using GeographicLib.
3. **Chassis Antenna Offsets**: Compensates for the physical offset between the GPS antenna mounting location and the rover center of rotation using `constants::NAVBOARD_EASTING_OFFSET`, `NAVBOARD_NORTHING_OFFSET`, and `NAVBOARD_ALTITUDE_OFFSET`.
4. **Kinematic Velocity and Heading Estimation**: Computes linear velocity from sequential GPS positions and angular velocity from sequential IMU heading differentials.
5. **Data Freshness and Failsafe**: Monitors telemetry latency via `IsOutOfDate()`, alerting the state machine if GPS packets drop for longer than `constants::NAVBOARD_MAX_GPS_DATA_AGE`.

---

## 2. Ingested Data Streams

The driver registers RoveComm callbacks for two primary telemetry packets:
- **`GPSLATLON`**: Contains double-precision latitude, longitude, altitude, and fix accuracy metrics.
- **`IMUDATA`**: Contains double-precision compass heading ($0^\circ$ to $360^\circ$ clockwise from North) and heading accuracy estimate in degrees.

---

## 3. Data Freshness Guard (`IsOutOfDate`)

GPS antennas can lose satellite lock, and network lines can experience dropped packets.
- Every incoming GPS packet updates `m_tmLastGPSUpdateTime`.
- Every incoming compass packet updates `m_tmLastCompassUpdateTime`.
- The `IsOutOfDate()` method checks:
  $$\Delta t_{\text{GPS}} = t_{\text{current}} - t_{\text{last GPS}}$$
  $$\Delta t_{\text{compass}} = t_{\text{current}} - t_{\text{last compass}}$$
  If $\Delta t_{\text{GPS}} > \text{constants::NAVBOARD\_MAX\_GPS\_DATA\_AGE}$ (default 3.0 seconds) or $\Delta t_{\text{compass}} > \text{constants::NAVBOARD\_MAX\_COMPASS\_DATA\_AGE}$, `IsOutOfDate()` returns true.
- The `NavigatingState` continuously polls `IsOutOfDate()`. If true, the rover halts and logs critical warnings, preventing blind runaway.

---

## 4. Concurrency and Thread Safety

Telemetry arrives on the `RoveCommUDP` background thread while multiple autonomy threads (`StateMachineHandler`, `GeoPlanner`, `VisualizationHandler`, `DriveBoard`) read navigation state simultaneously.
- Thread safety is enforced through granular `std::shared_mutex` instances:
  - `m_muLocationMutex`
  - `m_muHeadingMutex`
  - `m_muVelocityMutex`
  - `m_muAngularVelocityMutex`
- Callbacks acquire exclusive unique locks (`std::unique_lock`), while getter methods acquire shared read locks (`std::shared_lock`), ensuring high throughput without data races.

---

## 5. Public Interface Summary

```cpp
// Coordinate Getters
geoops::GPSCoordinate GetGPSData();
geoops::UTMCoordinate GetUTMData();

// Orientation & Kinematics
double GetHeading();
double GetHeadingAccuracy();
double GetVelocity();
double GetAngularVelocity();

// Freshness & Health
std::chrono::system_clock::duration GetGPSLastUpdateTime();
std::chrono::system_clock::duration GetCompassLastUpdateTime();
bool IsOutOfDate();
```
