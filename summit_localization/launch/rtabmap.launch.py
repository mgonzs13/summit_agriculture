# MIT License

# Copyright (c) 2025 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    use_gps_map = LaunchConfiguration("use_gps_map")
    use_gps_map_cmd = DeclareLaunchArgument(
        "use_gps_map",
        default_value="False",
        description="Use GPS map or RTAB-Map map frame",
    )

    slam_mode = LaunchConfiguration("slam_mode")
    slam_mode_cmd = DeclareLaunchArgument(
        "slam_mode",
        default_value="True",
        description="Whether to run in SLAM mode. Otherwise, it will run in Localization mode",
    )

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Wheather to launch rtabmapviz",
    )

    remappings = [
        ("rgb/image", "/robot/zed2/zed_node/rgb/image_rect_color"),
        ("rgb/camera_info", "/robot/zed2/zed_node/camera_info"),
        ("depth/camera_info", "/robot/zed2/zed_node/camera_info"),
        ("depth/image", "/robot/zed2/zed_node/depth/depth_registered"),
        ("scan_cloud", "/robot/top_3d_laser/points_filtered"),
        ("imu", "/robot/zed2/zed_node/imu/data"),
        ("gps/fix", "/robot/gps/fix"),
        ("odom", "odom"),
        ("goal", "goal_pose"),
    ]

    def run_rtabmap(context: LaunchContext, slam_mode, use_gps_map):
        slam_mode = eval(context.perform_substitution(slam_mode))
        use_gps_map = eval(context.perform_substitution(use_gps_map))

        if use_gps_map:
            remappings[-2] = ("odom", "/global_odom")

        parameters = [
            {
                "map_frame_id": "map",
                "frame_id": "robot/base_footprint",
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_scan_cloud": True,
                "approx_sync": True,
                "publish_tf": not use_gps_map,
                "use_sim_time": use_sim_time,
                "qos": 2,
                "qos_image": 1,
                "qos_camera_info": 1,
                "qos_imu": 2,
                "qos_gps": 1,
                # Hypotheses selection
                "Rtabmap/LoopGPS": "true",
                "Rtabmap/LoopThr": "0.11",
                "Rtabmap/CreateIntermediateNodes": "false",
                # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
                "Optimizer/Strategy": "2",
                "Optimizer/GravitySigma": "0.3",
                "RGBD/Enabled": "true",
                "RGBD/OptimizeMaxError": "3.0",
                "RGBD/OptimizeFromGraphEnd": "true",
                "RGBD/CreateOccupancyGrid": "true",
                "RGBD/LoopClosureIdentityGuess": "false",
                "RGBD/LocalBundleOnLoopClosure": "false",
                "RGBD/ProximityPathMaxNeighbors": "1",
                "VhEp/Enabled": "false",
                "GFTT/MinDistance": "7.0",
                "GFTT/QualityLevel": "0.001",
                "GFTT/BlockSize": "3",
                "GFTT/UseHarrisDetector": "true",
                "GFTT/K": "0.04",
                "BRIEF/Bytes": "64",
                # 0=Vis, 1=Icp, 2=VisIcp
                "Reg/Strategy": "2",
                "Reg/Force3DoF": "true",
                # Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
                "Vis/EstimationType": "1",
                "Vis/ForwardEstOnly": "true",
                # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
                "Vis/FeatureType": "8",
                "Vis/DepthAsMask": "true",
                "Vis/CorGuessWinSize": "40",
                "Vis/MaxFeatures": "0",
                "Vis/MinDepth": "0.0",
                "Vis/MaxDepth": "0.0",
                # ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).
                "Icp/Strategy": "1",
                "Icp/MaxTranslation": "0.2",
                "Icp/VoxelSize": "0.05",
                "Icp/DownsamplingStep": "1",
                "Icp/MaxCorrespondenceDistance": "0.1",
                "Icp/Iterations": "30",
                "Icp/Epsilon": "0.0",
                "Icp/CorrespondenceRatio": "0.1",
                "Icp/PointToPlane": "true",
                # 0=Features Matching, 1=Optical Flow
                "Vis/CorType": "0",
                # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
                "Vis/CorNNType": "1",
                # Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).
                "Grid/Sensor": "2",
                "Grid/DepthDecimation": "4",
                "Grid/RangeMin": "0.0",
                "Grid/RangeMax": "5.0",
                "Grid/MinClusterSize": "10",
                "Grid/MaxGroundAngle": "60",
                "Grid/NormalsSegmentation": "true",
                "Grid/NormalK": "20",
                "Grid/ClusterRadius": "0.2",
                "Grid/CellSize": "0.1",
                "Grid/FlatObstacleDetected": "false",
                "Grid/RayTracing": "true",
                "Grid/3D": "true",
                "Grid/MapFrameProjection": "true",
                "Grid/MaxGroundHeight": "0.1",
                "Grid/MaxObstacleHeight": "1.0",
                "Grid/DepthRoiRatios": "0.0 0.0 0.0 0.1",
                "GridGlobal/FootprintRadius": "0.4",
                "GridGlobal/UpdateError": "0.01",
                "GridGlobal/MinSize": "300.0",
                "GridGlobal/Eroded": "true",
                "GridGlobal/FloodFillDepth": "16",
            }
        ]

        arguments = ["--ros-args", "--log-level", "Warn"]

        if not slam_mode:
            parameters[0]["Mem/IncrementalMemory"] = "false"
            parameters[0]["Mem/InitWMWithAllNodes"] = "true"
            parameters[0]["RGBD/StartAtOrigin"] = "true"
        else:
            arguments.insert(0, "-d")

        return [
            Node(
                package="rtabmap_util",
                executable="obstacles_detection",
                name="lidar3d_obstacles_detection",
                output="log",
                parameters=parameters,
                remappings=remappings
                + [
                    ("cloud", "/robot/top_3d_laser/points_filtered"),
                    ("obstacles", "/lidar3d_obstacles"),
                    ("ground", "/lidar3d_ground"),
                ],
                arguments=arguments,
            ),
            Node(
                package="rtabmap_util",
                executable="obstacles_detection",
                name="rgbd_obstacles_detection",
                output="log",
                parameters=parameters,
                remappings=remappings
                + [
                    ("cloud", "/robot/zed2/zed_node/point_cloud/cloud_registered"),
                    ("obstacles", "/rgbd_obstacles"),
                    ("ground", "/rgbd_ground"),
                ],
                arguments=arguments,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=arguments,
            ),
        ]

    return LaunchDescription(
        [
            use_sim_time_cmd,
            slam_mode_cmd,
            launch_rtabmapviz_cmd,
            use_gps_map_cmd,
            OpaqueFunction(function=run_rtabmap, args=[slam_mode, use_gps_map]),
            Node(
                condition=IfCondition(launch_rtabmapviz),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                remappings=remappings,
                arguments=["--ros-args", "--log-level", "Warn"],
            ),
        ]
    )
