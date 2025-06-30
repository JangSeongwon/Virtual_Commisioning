from dataclasses import dataclass, field

@dataclass
class Robot:
    """
    Represents a robot with its configuration and capabilities.
    Contains methods for moving to home positions and checking convergence.
    """
    
    doosan_path: str = "/home/sms/catkin_ws/src/doosan-robot/common/imp"
    robot_id: str = "dsr01"
    robot_model: str = "m1509"
    # home_pos_cartesian: list = field(default_factory=lambda: [370.0, 30.0, 204.0, 0, 180.0, 0]) # [mm, deg], [x, y, z, Z1, Y, Z2]
    home_pos_cartesian: list = field(default_factory=lambda: [400.0, 30.0, 200.0, 0, 180.0, 0]) # [mm, deg], [x, y, z, Z1, Y, Z2]
    robot_vel: float = 8
    robot_acc: float = 50
    cartesian_pos_error_tolerance: float = 1e-3

    # CD = 9cm
    # H = 8cm
    # Sensor D = 7.4cm 
    # minimum EE hight must be 24.4
    # maximum EE hight must be 24.4 + ROI
    # RANGE = 1cm
    # Object_Surface = 4cm
    # Optimal Height = Object_Surface + CD + Sensor D + H/2 + RANGE -> Object_Surface + 21.4 
    # experimental offset = 1.5

@dataclass
class Planner:
    """
    Represents an object with a name and a list of poses.
    Contains dimensions for the object in the robot's coordinate system.
    difference from robot home pose
    """
    roi: dict = field(default_factory=lambda: {
        "x_min": -40.0,
        "x_max": +40.0,
        "y_min": -20.0,
        "y_max": +20.0,
        "z_min": 0.0,
        "z_max": 40.0
    })

    sensor_top_view_ori = [0, 0, 0]  # [Z1, Y, Z2], deg
    surface_to_sensor_distance = 90  # mm, distance from the surface to the sensor # CD(90) + H(80)/2 - OptimalRange(20) - ??
    xz_angle_list = [30, 45, 60, 75, 90, 105, 120, 135, 150]  # degrees
    
@dataclass
class Sensor:
    """
    Represents a sensor with a name and a list of poses.
    """
    ros_node_name: str = "gocator_pcd_collector"
    pcd_topic: str = "/gocator_profile_pcd"
    global_frame: str = "base_0"
    

@dataclass
class Config:
    """
    Configuration class for the robot controller.
    Contains settings for robot ID, model, home positions, velocities, and tolerances.
    """
    robot: Robot = field(default_factory=Robot)
    sensor: Sensor = field(default_factory=Sensor)
    planner: Planner = field(default_factory=Planner)