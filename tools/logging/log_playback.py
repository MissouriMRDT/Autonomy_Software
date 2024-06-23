import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import argparse
import csv
import re
from datetime import datetime

def parse_log_file(log_file_path):
    log_data = []
    # Read the log file into a pandas DataFrame
    with open(log_file_path, mode='r') as file:
        csvFile = csv.DictReader(file, delimiter='\t', fieldnames=["timestamp", "level", "thread", "trace", "message"])
        for line in csvFile:
            log_data.append(line)

    # Data for plotting
    gps_position = []
    gps_position_pattern = r"\(([^ ]+) lat, ([^ ]+) lon, ([^ ]+) alt\)"
    gps_compass = []
    gps_compass_pattern = r"Incoming Compass Data: ([\d\.]+)"
    
    rover_pose = []
    rover_pose_compass = []
    rover_pose_pattern = r"([\d\.]+) \(lat\), ([\d\.]+) \(lon\), ([\d\.]+) \(alt\), ([\d\.]+) \(degrees\), GNSS/VIO FUSED\? = (true|false)"

    accuracy = []
    accuracy_pattern = r"Incoming Accuracy Data: \(2D: ([\d\.]+), 3D: ([\d\.]+), Compass: ([\d\.]+), FIX_TYPE: ([\d\.]+)"
    
    fps = []
    fps_pattern = r"FPS: ([\d\.]+)"

    states = []
    states_pattern = r"Current State:\s*(.*)"
    drive_powers = []
    drive_powers_pattern = r"Driving at: \(([\d\.]+), ([\d\.]+)\)"

    waypoints = []
    clear_waypoints = []
    waypoints_pattern = r"\(lat: ([^ ]+), lon: ([^ ]+)\)"

    # Loop through the log data and package it into readable arrays.
    for line_entry in log_data:
        # Get timestamp and message.
        timestamp = datetime.strptime(line_entry["timestamp"].strip(","), "%Y-%m-%d %H:%M:%S.%f")
        message = line_entry["message"]

        # GPS Data line.
        if "GPS Data" in message:
            match = re.search(gps_position_pattern, message)
            if match:
                gps_position.append({"timestamp": timestamp, "lat": float(match.group(1)), "lon": float(match.group(2)), "alt": float(match.group(3))})

        # Rover Pose.
        if "Rover Pose" in message:
            match = re.search(rover_pose_pattern, message)
            if match:
                rover_pose.append({"timestamp": timestamp, "lat": float(match.group(1)), "lon": float(match.group(2)), "alt": float(match.group(3))})
                rover_pose_compass.append({"timestamp": timestamp, "heading": float(match.group(4))})

        # Compass Data.
        if "Compass Data" in message:
            match = re.search(gps_compass_pattern, message)
            if match:
                gps_compass.append({"timestamp": timestamp, "heading": float(match.group(1))})

        # Accuracy Data.
        if "Accuracy Data" in message:
            match = re.search(accuracy_pattern, message)
            if match:
                accuracy.append({"timestamp": timestamp, "2D": float(match.group(1)), "3D": float(match.group(2)), "compass": float(match.group(3)), "fix_type": int(match.group(4))})

        # FPS Data.
        if "Threads FPS" in message:
            match = re.findall(fps_pattern, message)
            if match:
                fps.append({"timestamp": timestamp, "main_process_fps": float(match[0]), "main_cam_fps": float(match[1]), "left_cam_fps": float(match[2]), "right_cam_fps": float(match[3]), "ground_cam_fps": float(match[4]), "main_detector_fps": float(match[5]), "left_detector_fps": float(match[6]), "right_detector_fps": float(match[7]), "state_machine_fps": float(match[8]), "rovecomm_udp_fps": float(match[9]), "rovecomm_tcp_fps": float(match[10])})
            match = re.findall(states_pattern, message)
            if match:
                states.append({"timestamp": timestamp, "state": match[0]})

        # Drivetrain Power Data.
        if "Driving at:" in message:
            match = re.search(drive_powers_pattern, message)
            if match:
                drive_powers.append({"timestamp": timestamp, "left_power": float(match.group(1)), "right_power": float(match.group(2))})

        # Waypoints Data.
        if "Waypoint" in message:
            match = re.search(waypoints_pattern, message)
            if match:
                waypoints.append({"timestamp": timestamp, "lat": float(match.group(1)), "lon": float(match.group(2))})

        # Clear WaypointHandler Data.
        if "Cleared" in message:
            clear_waypoints.append({"timestamp": timestamp})
    
    return {"gps_position": gps_position, "rover_pose": rover_pose, "rover_pose_compass": rover_pose_compass, "compass": gps_compass, "accuracy": accuracy, "fps": fps, "state": states, "drive_power": drive_powers, "waypoints": waypoints, "clear_waypoints": clear_waypoints}

def play_log(log_file_path):
    # Parse the log file
    parsed_data_dicts = parse_log_file(log_file_path)

    # Look at the first and last timestamp in the parsed data and use that as a start/end reference.
    start_datetime = parsed_data_dicts["fps"][0]["timestamp"]
    end_datetime = parsed_data_dicts["fps"][-1]["timestamp"]
    # Get the current start time.
    current_start_time = datetime.now()

    # Graph and graph data lists.
    plot_delay_fps = 333     # Millisecond interval to update plots in. Does not affect speed of log playback.
    plot_total_frames = int(plot_delay_fps * (end_datetime - start_datetime).total_seconds())
    fps_fig, fps_ax1 = plt.subplots()
    fps_values = []
    accuracy_fig, (acc_ax1, acc_ax2, acc_ax3) = plt.subplots(3, 1)
    accuracy_values = []
    compass_fig, comp_ax1 = plt.subplots()
    compass_values = []
    # Position plot.
    gps_values = []
    pose_values = []
    waypoints = []
    # 3D location fig for gps data.
    gps_pose_fig = plt.figure()
    gps_ax1 = gps_pose_fig.add_subplot(121, projection="3d")
    pose_ax1 = gps_pose_fig.add_subplot(122, projection="3d")
    # Drive power fig.
    drive_fig, drive_ax1 = plt.subplots()
    drive_ax1.set_ylim(-1, 1)
    drive_bars = drive_ax1.bar(["Left Power", "Right Power"], [0, 0])
    state_text = drive_fig.text(0.05, 1.0, "", verticalalignment='top', horizontalalignment='left')
    

    # Initialize plot lines
    fps_lines = [fps_ax1.plot([], [], label=label)[0] for label in [
        "main_process_fps", "main_cam_fps", "left_cam_fps", "right_cam_fps", "ground_cam_fps",
        "main_detector_fps", "left_detector_fps", "right_detector_fps", "statemachine_fps", "rovecomm_udp_fps", "rovecomm_tcp_fps"
    ]]
    acc_lines = [acc_ax1.plot([], [], label="2D")[0], acc_ax2.plot([], [], label="3D")[0], acc_ax3.plot([], [], label="compass")[0]]
    comp_lines = [comp_ax1.plot([], [], label=label)[0] for label in ["gps_compass", "pose_compass"]]
    gps_pose_lines = [gps_ax1.plot([], [], [], "b.", label="gps")[0], pose_ax1.plot([], [], [], label="pose")[0]]

    def animate_fps(i):
        # Get the time elapsed in the video in ms and add that onto the start datetime.
        current_time = datetime.now()
        delta = current_time - current_start_time
        current_video_datetime = start_datetime + delta
        # Get the first element of the same timestamp for all log data only accurate to the 0.01 second.
        new_fps = next((x for x in parsed_data_dicts["fps"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        if new_fps:
            fps_values.append(new_fps)
            # Only keep the last 100 accuracy values.
            if (len(fps_values) > 100):
                fps_values.pop(0)

            x = [value["timestamp"] for value in fps_values]
            y = [
                [value["main_process_fps"] for value in fps_values],
                [value["main_cam_fps"] for value in fps_values],
                [value["left_cam_fps"] for value in fps_values],
                [value["right_cam_fps"] for value in fps_values],
                [value["ground_cam_fps"] for value in fps_values],
                [value["main_detector_fps"] for value in fps_values],
                [value["left_detector_fps"] for value in fps_values],
                [value["right_detector_fps"] for value in fps_values],
                [value["state_machine_fps"] for value in fps_values],
                [value["rovecomm_udp_fps"] for value in fps_values],
                [value["rovecomm_tcp_fps"] for value in fps_values]
            ]
            for line, y_data in zip(fps_lines, y):
                line.set_data(x, y_data)
            fps_ax1.relim()
            fps_ax1.autoscale_view()
            fps_ax1.xaxis.set_major_locator(mdates.AutoDateLocator(interval_multiples=True))
            fps_ax1.xaxis.set_major_formatter(mdates.DateFormatter('%b %d, %Y %H:%M:%S'))

    fps_ani = FuncAnimation(fps_fig, animate_fps, interval=plot_delay_fps, frames=plot_total_frames, blit=True)

    def animate_accuracy(i):
        # Get the time elapsed in the video in ms and add that onto the start datetime.
        current_time = datetime.now()
        delta = current_time - current_start_time
        current_video_datetime = start_datetime + delta
        # Get the first element of the same timestamp for all log data only accurate to the 0.01 second.
        new_accuracy = next((x for x in parsed_data_dicts["accuracy"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        if new_accuracy:
            accuracy_values.append(new_accuracy)
            # Only keep the last 50 accuracy values.
            if (len(accuracy_values) > 50):
                accuracy_values.pop(0)

            x = [value["timestamp"] for value in accuracy_values]
            y1 = [value["2D"] for value in accuracy_values]
            y2 = [value["3D"] for value in accuracy_values]
            y3 = [value["compass"] for value in accuracy_values]

            acc_lines[0].set_data(x, y1)
            acc_lines[1].set_data(x, y2)
            acc_lines[2].set_data(x, y3)

            for ax in [acc_ax1, acc_ax2, acc_ax3]:
                ax.relim()
                ax.autoscale_view()
                ax.xaxis.set_major_locator(mdates.AutoDateLocator(interval_multiples=True))
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%b %d, %Y %H:%M:%S'))

    accuracy_ani = FuncAnimation(accuracy_fig, animate_accuracy, interval=plot_delay_fps, frames=plot_total_frames, blit=True)

    def animate_compass(i):
        # Get the time elapsed in the video in ms and add that onto the start datetime.
        current_time = datetime.now()
        delta = current_time - current_start_time
        current_video_datetime = start_datetime + delta
        # Get the first element of the same timestamp for all log data only accurate to the 0.01 second.
        new_compass = next((x for x in parsed_data_dicts["compass"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        new_pose_compass = next((x for x in parsed_data_dicts["rover_pose_compass"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        if new_compass and new_pose_compass:
            new_compass_values = {"timestamp": new_compass["timestamp"], "heading": new_compass["heading"], "pose_heading": new_pose_compass["heading"]}
            compass_values.append(new_compass_values)

            x = [value["timestamp"] for value in compass_values]
            y = [[value["heading"] for value in compass_values],
                 [value["pose_heading"] for value in compass_values]
            ]

            for line, y_data in zip(comp_lines, y):
                line.set_data(x, y_data)
            comp_ax1.relim()
            comp_ax1.autoscale_view()
            comp_ax1.xaxis.set_major_locator(mdates.AutoDateLocator(interval_multiples=True))
            comp_ax1.xaxis.set_major_formatter(mdates.DateFormatter('%b %d, %Y %H:%M:%S'))

    compass_ani = FuncAnimation(compass_fig, animate_compass, interval=plot_delay_fps, frames=plot_total_frames, blit=True)

    def animate_gps_pose(i):
        # Get the time elapsed in the video in ms and add that onto the start datetime.
        current_time = datetime.now()
        delta = current_time - current_start_time
        current_video_datetime = start_datetime + delta
        # Get the first element of the same timestamp for all log data only accurate to the 0.01 second.
        new_gps = next((x for x in parsed_data_dicts["gps_position"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        new_pose = next((x for x in parsed_data_dicts["rover_pose"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        new_waypoint = next((x for x in parsed_data_dicts["waypoints"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        cleared_waypoints = next((x for x in parsed_data_dicts["clear_waypoints"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        # Discard first few seconds of data.
        if (delta.seconds >= 5):
            if new_gps:
                gps_values.append(new_gps)

                x = [value["lat"] for value in gps_values]
                y = [value["lon"] for value in gps_values]
                z = [value["alt"] for value in gps_values]

                # gps_pose_lines[0].set_data_3d(x, y, z)
                gps_pose_lines[0].set_data(x, y)
                gps_pose_lines[0].set_3d_properties(z)

                if (len(x) > 5 and len(y) > 5 and len(z) > 5):
                    gps_ax1.set(xlim3d=(min(x), max(x)), xlabel='X')
                    gps_ax1.set(ylim3d=(min(y), max(y)), ylabel='Y')
                    gps_ax1.set(zlim3d=(min(z), max(z)), zlabel='Z')
            
            if new_pose:
                pose_values.append(new_pose)

                x = [value["lat"] for value in pose_values]
                y = [value["lon"] for value in pose_values]
                z = [value["alt"] for value in pose_values]

                gps_pose_lines[1].set_data_3d(x, y, z)

                if (len(x) > 5 and len(y) > 5 and len(z) > 5):
                    pose_ax1.set(xlim3d=(min(x), max(x)), xlabel='X')
                    pose_ax1.set(ylim3d=(min(y), max(y)), ylabel='Y')
                    pose_ax1.set(zlim3d=(min(z), max(z)), zlabel='Z')

        # Add vertical lines to the graph when waypoints are added.
        if new_waypoint:
            waypoints.append(new_waypoint)
            
        for waypoint in waypoints:
            gps_ax1.plot([waypoint["lat"], waypoint["lat"]], [waypoint["lon"], waypoint["lon"]], [min(z), max(z)], color='r', linestyle='--', label='waypoint')

        # # The waypoints have been cleared.
        # if cleared_waypoints:


    gps_pose_ani = FuncAnimation(gps_pose_fig, animate_gps_pose, interval=plot_delay_fps, frames=plot_total_frames, blit=True)

    def animate_drive_powers(i):
        # Get the time elapsed in the video in ms and add that onto the start datetime.
        current_time = datetime.now()
        delta = current_time - current_start_time
        current_video_datetime = start_datetime + delta
        # Get the first element of the same timestamp for all log data only accurate to the 0.01 second.
        new_state = next((x for x in parsed_data_dicts["state"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        new_drive_powers = next((x for x in parsed_data_dicts["drive_power"] if int(x["timestamp"].timestamp() * 10) == int(current_video_datetime.timestamp() * 10)), None)
        if new_drive_powers:
            drive_bars[0].set_height(new_drive_powers["left_power"])
            drive_bars[1].set_height(new_drive_powers["right_power"])

        if new_state:
            state_text.set_text(f"Current State: {new_state['state']}")

    drive_power_ani = FuncAnimation(drive_fig, animate_drive_powers, interval=plot_delay_fps, frames=plot_total_frames, blit=True)

    # Adjust layout and show the plots
    fps_ax1.set_title("Threading Performance")
    acc_ax1.set_title("GPS Accuracy")
    comp_ax1.set_title("Compass Heading")
    gps_ax1.set_title("GPS Lat/Lon")
    pose_ax1.set_title("Rover Pose")
    drive_ax1.set_title("Drivetrain Powers")
    fps_ax1.legend(loc="upper left")
    acc_ax1.legend(loc="upper left")
    acc_ax2.legend(loc="upper left")
    acc_ax3.legend(loc="upper left")
    comp_ax1.legend(loc="upper left")
    gps_ax1.legend(loc="upper left")
    pose_ax1.legend(loc="upper left")

    # Plot and show graphs.
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Animated log data visualization.")
    parser.add_argument("--log_file", type=str, required=True, help="Path to the log file.")
    
    args = parser.parse_args()

    print(args.log_file)
    
    play_log(args.log_file)

