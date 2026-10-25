"""Single source of truth for the camera and model names (used by the world generator, bridge and SLAM)."""
import math

WORLD_NAME = "rtlf_urban"
MODEL_NAME = "rtlf_x500_rgbd_0"      # instance name in the world; PX4 attaches to it
MODEL_URI = "rtlf_x500_rgbd"

WIDTH, HEIGHT = 320, 240
HFOV = 1.2                            # rad
NEAR, FAR = 0.2, 100.0                 # m
RATE_HZ = 15
SENSOR_TOPIC = "rtlf/rgbd"            # gz topics: /rtlf/rgbd/image and /rtlf/rgbd/depth_image
# camera position relative to the vehicle origin, body frame FRD (forward, right, down) [m]
LEVER_FRD = (0.12, 0.0, -0.10)
CAM_PITCH_DEG = 25.0   # camera pitched DOWN about the body y axis

FX = (WIDTH / 2.0) / math.tan(HFOV / 2.0)
FY = FX
CX = WIDTH / 2.0
CY = HEIGHT / 2.0
DEPTH_SCALE_PNG = 400.0              # uint16 PNG depth units per metre in recorded datasets (max 65 m)
