# Maize GPS Viewer

A suite of tools for maize detection, GPS tracking, and visualization in agricultural robotics.

## Table of Contents

- Overview
- Components
  - Corn Tracker
  - GPS Line Generator
  - Maize Viewer (FLOR)
- Installation
- Usage
- Data Format

## Overview

The Maize GPS Viewer project provides tools for precision agriculture, specifically focusing on corn field monitoring, route planning, and data visualization using ROS 2 and geospatial technologies.

## Components

### Corn Tracker

[`summit_agriculture/maize_gps_viewer/corn_tracker.py`](summit_agriculture/maize_gps_viewer/corn_tracker.py ) is a ROS 2 node that helps robots detect corn plants in agriculture fields. It:

- Monitors the robot's camera feed
- Uses GPS position data
- Processes 3D object detections from a YOLO model
- Calculates the real-world GPS location of detected corn plants
- Saves cropped images of each detected corn plant
- Records data in a CSV file (ID, GPS coordinates, image filename, confidence)
- Avoids duplicate detections

This creates a comprehensive map of corn locations throughout a field, with visual references for each detected plant.

### GPS Line Generator

[`summit_agriculture/maize_gps_viewer/gps_line_generator.py`](summit_agriculture/maize_gps_viewer/gps_line_generator.py ) generates efficient navigation paths within GPS-defined field boundaries:

- Creates parallel line trajectories for field coverage
- Converts between GPS and Cartesian coordinates
- Supports various path directions:
  - Perpendicular to the longest side
  - Parallel to the longest side
  - Perpendicular to the shortest side
  - Parallel to the shortest side
- Adjustable line spacing and point density
- Exports paths in YAML and CSV formats

Perfect for creating efficient coverage patterns for agricultural operations like planting, spraying, or inspection.

### Maize Viewer (FLOR)

**FLOR** (**F**ichas con **L**ocalización, **O**bservaciones y **R**eferencia visual) is a Streamlit-based visualization tool:

- Interactive map display with multiple layer options:
  - ESRI Satellite
  - Google Satellite
  - Google Hybrid
  - OpenStreetMap
- Plant markers with detailed popups
- Image preview for each detected plant
- "Super Zoom" capability for detailed inspection
- Search and filter functionality
- Export options for sharing data
- Measurement tools for distance calculation

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your-username/maize_gps_viewer.git
cd maize_gps_viewer
```

2. Install dependencies:
```bash
# For ROS 2 components
pip install rclpy cv_bridge

# For GPS Line Generator
pip install shapely geopy pyyaml

# For Maize Viewer
pip install streamlit pandas folium streamlit-folium pillow requests
```

## Usage

### Corn Tracker

Run the ROS 2 node:
```bash
ros2 run maize_gps_viewer corn_tracker
```

Ensure the following topics are available:
- `/robot/gps/fix` (NavSatFix)
- `/yolo/detections_3d` (DetectionArray)
- `/robot/zed2/zed_node/rgb/image_rect_color` (Image)

### GPS Line Generator

Generate navigation paths:
```bash
python gps_line_generator.py
```

Modify the script parameters to adjust:
- Line spacing
- Point density
- Direction of lines
- Input/output files

### Maize Viewer (FLOR)

Launch the Streamlit application:
```bash
streamlit run maize_viewer.py
```

Then upload your CSV file with plant data (columns: id, latitud, longitud, imagen).

## Data Format

The system uses a standard CSV format:

```
id,latitud,longitud,imagen,confidence
corn_1,42.12345678,-71.12345678,corn_1.png,0.95
corn_2,42.12345679,-71.12345677,corn_2.png,0.87
```