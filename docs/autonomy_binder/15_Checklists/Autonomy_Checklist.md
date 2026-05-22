# Autonomy Pre-Flight Checklist

*This checklist covers all the critical steps required to build, test, and deploy the autonomy software onto the rover during a competition or major test session.*

## Jetson Setup (On the Rover)

- [ ] **Check out the correct branch.**
  - `git checkout <branch-name>`
- [ ] **Fetch and pull the latest code.**
  - `git fetch`
  - `git pull`
- [ ] **Ensure that you have the correct constants set.**
  - In `CMakeLists.txt`, make sure that `BUILD_SIM_MODE` is `OFF`.
  - In `AutonomyConstants.cpp`, ensure that the correct database (.db) file is in the LiDAR database path: `LIDAR_HANDLER_DB_PATH`.
  - Also ensure that the correct tag/object model paths are set: `TAGDETECT_TORCH_MODEL` and `OBJECTDETECT_TORCH_MODEL`.
- [ ] **Build the code.**
  - Remove the build directory if it exists through GUI or command line.
  - `mkdir build && cd build`
  - `cmake -DCMAKE_BUILD_TYPE=Release .. && make -j12`
- [ ] **Test the code on the Jetson off-rover before putting it on-rover.**
  - Plug a ZED or two in.
  - Run the executable: `./Autonomy_Software`
  - *(If the executable generated is `./Autonomy_Software_Sim`, change the `BUILD_SIM_MODE` value in `CMakeLists.txt` and rebuild).*
- [ ] **Ensure that the ZEDs you plugged in successfully opened.**
  - Check the console logs. If they did not open, restart the code or rebuild the container. You may also have to re-plug in the ZED cables.
  - Type `lsusb` in the terminal to see if they populate. You should see **two** devices per ZED camera.
- [ ] **Verify Framerates (FPS).**
  - Press `f` in the terminal while the code is running. If the FPS output all looks good and stable, you are ready to test on-rover!
  - *(You can test detection models directly on the Jetson with a ZED without the rover if you need to).*
- [ ] **Turn off WiFi / Airplane Mode.**
  - **MAKE SURE IT IS IN AIRPLANE MODE OR ELSE THE SOLAR FLARES WILL DECLARE OUR LOSS AT URC!** (Disconnect from external networks to prevent interference/packet loss).
- [ ] **Turn off the Jetson and mount it physically on the rover.**

---

## Autonomy on Basestation Computer

### Setup and Building

- [ ] **Connect to the Rover.**
  - Open terminal and SSH into Jetson: `ssh pigeon@192.168.3.100`
  - *(Password is `nandgate`)*
- [ ] **Start the Dev Container (if not already running).**
  - `cd Documents`
  - `./Helpful_Run_Autonomy` (Docker setup script)
- [ ] **Run the Autonomy Software.**
  - `cd build`
  - `./Autonomy_Software`
- [ ] **(Optional) Open VSCode remotely to change code.**
  - Connect via SSH in VSCode. You can edit code here, but it is recommended to run the executable from the standard SSH terminal.
  - If you save code and get a permission error, run this command in the VSCode terminal: `sudo chown -R pigeon:pigeon ./Autonomy_Software`
- [ ] **View Output / Logs.**
  - After running autonomy, go to the `logs/` directory in VSCode to see the rover’s search patterns, paths, and recorded video feeds.
- [ ] **Rebuilding after a quick fix.**
  - If you need to rebuild code after editing, run `make -j12` in the terminal inside the `build` folder.
  - If CMake errors occur, clean it: `rm -r build && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j12`

### Basestation API & GUI

- [ ] **Open up the Autonomy layout from the GUI dropdown.**
- [ ] **Input Waypoints.**
  - In the "Waypoints" section, input the GPS coordinates, name, radius, and objective IDs.
  - Use ID **`-2`** for a Mallet objective.
  - Use ID **`-3`** for a Water Bottle objective.
  - *(You can also right-click directly on the GUI map to drop a waypoint).*
- [ ] **Start Autonomous Navigation.**
  - To get the Rover to drive autonomously, click on the waypoint you want to navigate to in the queue, then click **"Add Leg"**.
  - Ensure that the waypoint successfully appears in the Autonomy terminal output.
  - Click the **Start** button once it does to transition out of `eIdle`.
- [ ] **Manage the Queue.**
  - You can delete waypoints by clearing the queue if a mistake is made or an abort is required.
- [ ] **Monitor Success.**
  - Once you see the Operator State indicator turn **Green** (and the Rover's LED strip flashes green), you know that the Rover has successfully navigated to the waypoint or detected the objective.

---

## Bug Fixing & General Knowledge

- **Not detecting ZED or Bad positional tracking?**
  - Unplug and securely re-plug the USB-C cord from the Jetson to the ZED camera. Restart the autonomy software.
- **Where is the rover going / Why is it doing that?**
  - Open up the `logs/` folder and read the latest `.log` file! Look for state transitions and `GeoPlanner` warnings.
- **Is our heading off?**
  - Press `p` in the SSH terminal while autonomy is running. Compare the printed Autonomy heading with the raw NAV component heading on the basestation.
- **Are we seeing tags or objects?**
  - Press `t` to dump AR tag detections to the terminal.
  - Press `m` to dump Neural Network object detections to the terminal.
- **Is the software lagging?**
  - After `Autonomy_Software` is running, press `f` to view the FPS menu and verify that the detectors, cameras, and state machine are hitting their maximum iteration speeds.
