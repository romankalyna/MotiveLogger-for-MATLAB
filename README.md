# MarkerLogger: MATLAB OptiTrack Data Logger for Motive Tracker 2.2.0

## Overview

**MarkerLogger** is a MATLAB-based tool for logging and visualizing tracking data from Motive Tracker 2.2.0.  
It supports up to **2 rigid bodies** and **6 markers**, saving their positions and orientations in a robust CSV format—even if Motive Tracker crashes during a session.

## Features

- Logs position and rotation (XYZ + Euler angles) for 2 rigid bodies.
- Tracks XYZ positions for 6 markers (3 per rigid body).
- Converts Motive coordinates to a custom "My Robot" coordinate system:
    - My Robot X = Motive Z
    - My Robot Y = Motive X
    - My Robot Z = Motive Y
- RB2 is treated as the origin for visualization and data export (units: millimeters).
- Calculates and logs the distance between RB1 and RB2.
- On-demand logging using the `update` command—ideal for projects that do not require real-time logging.
- Automatically saves all logged data to CSV, even if Motive or MATLAB crashes mid-session.
- Visualization window displays trajectory, orientation, and marker history.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/romankalyna/motive-matlab-logger.git
   ```
2. Add the folder to your MATLAB path.

## Usage Example

```matlab
% Initialize the logger
MarkerLogger('init');

% Log marker and rigid body data (call periodically)
MarkerLogger('log');

% Update visualization and flush the latest data (on-demand)
MarkerLogger('update');

% Save data to CSV file
MarkerLogger('save');

% Close logger and clean up
MarkerLogger('close');
```

## Data Format

The output CSV file contains:
- Sample index and timestamp
- RB1/RB2 positions (MyRobot XYZ, mm)
- RB1/RB2 quaternions (original Motive orientation)
- RB1/RB2 Euler angles (MyRobot frame, degrees)
- Distance between RB1 and RB2 (meters)
- XYZ positions of all 6 markers (MyRobot frame, mm)

## Reliability

**Crash-safe:** Data is written to disk and completed even if Motive Tracker or MATLAB crashes during operation. No data is lost.

## Requirements

- MATLAB R2020b or newer
- Motive Tracker 2.2.0


For issues and feature requests, [open an issue](https://github.com/romankalyna/motive-matlab-logger/issues).
