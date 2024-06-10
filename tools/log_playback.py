import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import argparse
import csv
import re
from datetime import datetime

def parse_log_file(log_file_path):
    log_data = []

    # Read the log file into a pandas DataFrame
    with open(log_file_path, mode ='r') as file:    
       csvFile = csv.DictReader(file, delimiter='\t', fieldnames=["timestamp", "level", "thread", "trace", "message"])
       for line in csvFile:
            log_data.append(line)
    
    return log_data

def play_video_with_log(video_file_path, log_file_path):
    # Open the video file
    cap = cv2.VideoCapture(video_file_path)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # Parse the log file
    log_data = parse_log_file(log_file_path)
    
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Play video with synchronized log data visualization.")
    parser.add_argument("--video_file", type=str, help="Path to the video file.")
    parser.add_argument("--log_file", type=str, help="Path to the log file.")
    
    args = parser.parse_args()

    print(args.video_file, args.log_file)
    
    play_video_with_log(args.video_file, args.log_file)
