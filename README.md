# MarkerLogger

MarkerLogger is a robust MATLAB tool for logging and visualizing tracking data from Motive Tracker 2.2.0. Designed for projects involving motion capture, robotics, and biomechanics, MarkerLogger captures positions, rotations, and marker data for two rigid bodies and their associated markers, transforms coordinates to a custom "My Robot" frame, displays interactive 3D graphs, and exports all data to crash-safe CSV files.

---

## Features

- **Logs Two Rigid Bodies:** Captures position (XYZ) and rotation (quaternions and Euler angles) for two rigid bodies (RB1 and RB2).
- **Tracks Six Markers:** Logs XYZ coordinates for six markers (three per rigid body).
- **Custom Coordinate Transformation:** Converts Motive Tracker coordinates to "My Robot" frame:
  - My Robot X = Motive Z
  - My Robot Y = Motive X
  - My Robot Z = Motive Y
- **RB2 as Origin:** Visualizes and logs all positions relative to RB2, in millimeters.
- **Distance Calculation:** Computes and records the distance between RB1 and RB2 for each sample.
- **Interactive Graphs:** Real-time 3D visualization window shows marker trajectories, rigid body positions and orientations, color-coded axes, and live distance display.
- **Orientation History:** Plots Euler angles (Roll, Pitch, Yaw) for both rigid bodies over time.
- **On-Demand Logging:** Supports stepwise logging via the `update` command—suitable for non-real-time applications.
- **Crash-Safe Data:** Ensures all logged data is safely written to CSV even if Motive or MATLAB crashes.
- **Flexible Workflow:** Use simple commands to initialize, log, update, save, and close the logger and its visualization.

---

## Requirements

- MATLAB R2020b or newer
- Motive Tracker 2.2.0
- **NatNet.p MATLAB SDK file:**  
  To communicate with Motive Tracker from MATLAB, you must have the NatNet SDK properly installed and the `NatNet.p` file present in your MATLAB path.  
  - The NatNet SDK provides the interface for real-time data streaming from Motive to MATLAB.
  - You can download the NatNet SDK from [OptiTrack's website](https://optitrack.com/products/natnet-sdk/).
  - Place `NatNet.p` (and any required dependencies) in your project folder or somewhere on your MATLAB path.

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/romankalyna/MarkerLogger.git
   ```
2. Add the folder to your MATLAB path:
   ```matlab
   addpath('path_to_MarkerLogger_folder');
   ```
3. **NatNet SDK:**  
   - Download the NatNet SDK and copy `NatNet.p` to your MATLAB project directory.
   - Ensure any other required NatNet files are available and your MATLAB can call NatNet functions.

---

## Usage

Below is a typical workflow for logging and visualizing Motive Tracker data:

```matlab
% Initialize the logger and visualization
MarkerLogger('init');

% Log marker and rigid body data (call in your data acquisition loop)
MarkerLogger('log');

% Update the visualization and flush the latest data (on-demand)
MarkerLogger('update');

% Save all logged data to a CSV file
MarkerLogger('save', 'my_experiment_data.csv');

% Close the logger and clean up resources
MarkerLogger('close');
```

**Typical workflow:**  
- Call `'init'` once to set up.
- Call `'log'` each time you want to record a data point.
- Call `'update'` to refresh the graph and process the latest data.
- Call `'save'` to write data to CSV (can be done any time, even after a crash).
- Call `'close'` to release resources (the visualization window remains open).

---

## Data Format

The exported CSV file contains, for each sample:
- **SampleIdx** and timestamp (approximate, based on sample count)
- **RB1 and RB2 positions** (`MyRobot X`, `Y`, `Z` in mm)
- **RB1 and RB2 orientations:** Quaternions (original Motive) and Euler angles (MyRobot frame, degrees)
- **Distance between RB1 and RB2** (meters)
- **Marker positions:** XYZ for all six markers (MyRobot frame, mm)

**Example CSV header:**
```
SampleIdx,Time_s_approx,
RB1_MyRobotX_mm,RB1_MyRobotY_mm,RB1_MyRobotZ_mm,
RB1_qX_motive,RB1_qY_motive,RB1_qZ_motive,RB1_qW_motive,
RB1_MyRobotRoll_deg,RB1_MyRobotPitch_deg,RB1_MyRobotYaw_deg,
RB2_MyRobotX_mm,RB2_MyRobotY_mm,RB2_MyRobotZ_mm,
RB2_qX_motive,RB2_qY_motive,RB2_qZ_motive,RB2_qW_motive,
RB2_MyRobotRoll_deg,RB2_MyRobotPitch_deg,RB2_MyRobotYaw_deg,
Dist_RB1_RB2_m_Current,
M1_MyRobotX_mm,M1_MyRobotY_mm,M1_MyRobotZ_mm, ... up to M6
```

---

## Visualization

- **Marker trajectories:** Each marker’s motion is shown as a colored line in the 3D plot.
- **Rigid body frames:** RB1 and RB2 positions, orientations, and axes are visualized in real time.
- **RB2 as origin:** All data is plotted and logged with RB2 as the coordinate origin.
- **Euler angles:** Subplots show the roll, pitch, and yaw history for both rigid bodies.
- **Live info:** The graph window displays the current distance between RB1 and RB2 and the latest positions.

---

## Reliability

- **Crash-safe:** All logged data is preserved and written to CSV, even if Motive Tracker or MATLAB crashes during the experiment.
- **On-demand:** Data can be logged and visualized stepwise, making it suitable for batch or non-realtime projects.

---

## Example Applications

- Motion capture experiments
- Robotics and biomechanics research
- Real-time or stepwise rigid body tracking
- Marker-based trajectory analysis

---

## Support

For questions, suggestions, or bug reports, please [open an issue](https://github.com/romankalyna/MarkerLogger/issues).

---

## Credits

Developed by [Roman Kalyna](https://github.com/romankalyna).
