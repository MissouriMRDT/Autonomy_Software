# Navigation Board Driver

The `NavigationBoard` driver is arguably the most critical data ingestion pipeline in the autonomy software. It interfaces with the rover's GPS and IMU hardware to maintain the "Global Ground Truth" of where the rover is located and which direction it is pointing.

## Primary Responsibilities
1. **Data Ingestion**: It subscribes to data streams from the physical Navigation board to receive continuous updates on GPS Latitude/Longitude and IMU compass heading.
2. **Coordinate Transformation**: It automatically converts incoming GPS coordinates into the 2D Cartesian UTM (Universal Transverse Mercator) coordinate system to make distance math significantly easier for the path planners.
3. **Data Freshness and Velocity**: It calculates the rover's velocity and tracks the age of the incoming data to ensure the autonomy software doesn't act on stale positioning.

## Algorithm Explanation

### 1. Subscription via RoveComm
When the driver initializes, it constructs a RoveComm UDP `SUBSCRIBE` packet and fires it at the Navigation Board's IP address. This tells the physical microcontroller to start spamming the Jetson with its telemetry data.

### 2. Updating Data
The class registers several RoveComm callbacks (e.g., `UpdateGPSDataCallback`, `UpdateCompassDataCallback`).
- When a `GPSLATLON` packet arrives, it extracts the latitude, longitude, altitude, and lock accuracy (e.g., standard vs RTK differential).
- It applies offsets (`NAVBOARD_EASTING_OFFSET`, etc.) to account for the physical placement of the GPS antenna relative to the true center of the rover.
- The data is wrapped in a `geoops::GPSCoordinate` struct. When requested by the rest of the software, it uses the `geoops` utility library to project this GPS point into a `geoops::UTMCoordinate`.
- When an `IMUDATA` (heading) packet arrives, it stores the heading and calculates the *Angular Velocity* based on how much the heading changed since the last packet and how much time has elapsed.

### 3. Out-Of-Date Protection (`IsOutOfDate`)
GPS receivers can lose lock, or the ethernet cable could get unplugged. The driver tracks the `m_tmLastGPSUpdateTime`. If the time since the last packet exceeds `NAVBOARD_MAX_GPS_DATA_AGE` (usually 3 seconds), the `IsOutOfDate()` method returns true. The State Machine continuously polls this method, and if it returns true, the rover will immediately halt and enter the `eIdle` state to prevent driving blind.

## Inputs and Outputs
- **Inputs**:
  - `GPSLATLON`: RoveComm UDP packets from the Navigation Board containing GPS data.
  - `IMUDATA`: RoveComm UDP packets containing the compass heading (0-360 degrees).
- **Outputs**:
  - Exposes thread-safe getters like `GetGPSData()`, `GetUTMData()`, `GetHeading()`, `GetVelocity()`, and `GetAngularVelocity()` for the rest of the autonomy software to use.

## Usage in State Machine
The `NavigationBoard` is polled constantly.
- The `GeoPlanner` requests the current UTM coordinate to use as the starting point for A* pathfinding.
- The `DriveBoard` requests the current Heading to calculate the error in the PID steering controller.
- The `StuckState` checks the Velocity and Angular Velocity to determine if the rover has successfully un-stuck itself.
