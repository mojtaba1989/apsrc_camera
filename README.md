# Webcam Node

## Overview
The **webcam_node** is a ROS node designed to capture video streams from a webcam, save the video as an `.mp4` file, and publish relevant information such as recording details and elapsed time as ROS messages. It uses **OpenCV** for video capture and encoding.

The node dynamically adjusts the resolution, frame rate, and device ID based on ROS parameters. It also generates video filenames with timestamps and saves them to a specified directory.

---

## Features
- 📹 **Captures video** from a webcam device.
- 💾 **Saves video** as an `.mp4` file with a timestamped filename.
- 📊 **Publishes elapsed time** during recording to a ROS topic (`apsrc_msgs/ElapsedTime`).
- 📜 **Publishes recording information** like file path, resolution, FPS to a ROS topic (`apsrc_msgs/RecInfo`).
- 🔧 **Configurable parameters** through the ROS launch file.

---

## Topics
The node publishes the following topics:

| Topic                           | Message Type             | Description                                                           |
|---------------------------------|-------------------------|----------------------------------------------------------------------|
| `/camera_name/elapsed_time`      | `apsrc_msgs/ElapsedTime` | Publishes elapsed time (in nanoseconds) during recording.            |
| `/camera_name/info`              | `apsrc_msgs/RecInfo`    | Publishes video recording info like file path, resolution, FPS, etc. |

---

## Parameters
The following parameters can be configured in the launch file:

| Parameter     | Type     | Default     | Description                                  |
|---------------|---------|-------------|-----------------------------------------------|
| `camera_name` | `string` | `webcam`    | Name of the camera.                         |
| `device_id`   | `int`    | `0`         | The device ID of the webcam.                 |
| `width`       | `int`    | `640`       | Width of the video frame.                   |
| `height`      | `int`    | `480`       | Height of the video frame.                  |
| `fps`         | `int`    | `15`        | Frames per second for video capture.        |
| `dir`         | `string` | `/tmp`      | Directory to save the recorded video.       |

---

## File Naming Convention
The recorded video files are named using the following convention:

```
<camera_name>_YYYY_MM_DD_HH_MM_SS.mp4
```

### Example
A file recorded on **March 9, 2025, at 12:30:45 PM** from a camera named `webcam` will be saved as:

```
webcam_2025_03_09_12_30_45.mp4
```

The file will be saved in the directory specified by the `dir` parameter.

---

## Usage

### 1. Build the Package
First, place the package in your ROS workspace and build it using:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 2. Run the Node
Launch the node using the provided launch file:

```bash
roslaunch apsrc_camera webcam.launch
```

The node will:
- Capture video from the webcam.
- Save the video to the directory specified in the `dir` parameter.
- Publish recording information and elapsed time to the respective ROS topics.

---

## Using Current Directory as Save Location
The launch file provides an option to use the current working directory (`pwd`) as the save location for recorded videos. To enable this, modify the launch file to:

```xml
<arg name="dir" default="$(env pwd)"/>
```

Before launching the node, ensure you export the `pwd` environment variable like this:

```bash
export pwd=$(pwd)
roslaunch apsrc_camera webcam.launch
```

This will save the recorded videos in the current working directory.

If you do not set the `pwd` environment variable, the launch will fail with an error stating that the variable is not set.

---

## Expected Output
- The video file will be saved as:
  ```
  /<dir>/<camera_name>_YYYY_MM_DD_HH_MM_SS.mp4
  ```
- You can verify the elapsed time with:
  ```bash
  rostopic echo /<camera_name>/elapsed_time
  ```
- You can check recording info with:
  ```bash
  rostopic echo /<camera_name>/info
  ```

---

## Troubleshooting

### Video Capture Fails
If the node fails to open the camera, verify the device ID (`device_id` parameter). You can list available devices using:

```bash
v4l2-ctl --list-devices
```

### Permission Issues
If the video cannot be saved, ensure the node has write permission to the directory specified in `dir`. You can change permissions using:

```bash
sudo chmod -R 777 /your/dir
```

### Node Crashes on Launch
If the node crashes during launch, confirm the webcam is not being used by another process and the device ID is correct.

You can also check ROS logs for more information:

```bash
roslaunch apsrc_camera webcam.launch --screen
```

---

## License
This package is released under the **MIT License**. You are free to use, modify, and distribute it under the terms of this license.

