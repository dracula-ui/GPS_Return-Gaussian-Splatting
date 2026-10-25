"""Frame conventions and small geometry helpers (numpy only).

Local frame: PX4 NED (x north, y east, z down) with the origin at the take-off point.
Vehicle body: FRD (x forward, y right, z down); attitude quaternion q = body -> NED (PX4 convention).
Camera optical frame: x right, y down, z forward (OpenCV / MonoGS). The camera looks along body +x.
"""
import numpy as np

from . import sim_params as _SP

_R_LEVEL = np.array([[0.0, 0.0, 1.0],
                     [1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0]])
_th = np.radians(_SP.CAM_PITCH_DEG)
_R_PITCH = np.array([[np.cos(_th), 0.0, -np.sin(_th)],
                     [0.0, 1.0, 0.0],
                     [np.sin(_th), 0.0, np.cos(_th)]])
R_BC = _R_PITCH @ _R_LEVEL


def quat_to_R(w, x, y, z):
    n = (w * w + x * x + y * y + z * z) ** 0.5
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """Rotation matrix -> (w, x, y, z)."""
    t = np.trace(R)
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def camera_pose_from_vehicle(p_ned, q_nb, lever_frd):
    """Vehicle position (NED) + attitude (w,x,y,z body->NED) -> camera position and camera->NED rotation."""
    R_nb = quat_to_R(*q_nb)
    return np.asarray(p_ned, float) + R_nb @ np.asarray(lever_frd, float), R_nb @ R_BC


def vehicle_from_camera(p_cam_ned, q_nb, lever_frd):
    R_nb = quat_to_R(*q_nb)
    return np.asarray(p_cam_ned, float) - R_nb @ np.asarray(lever_frd, float)


def w2c_from_cam_pose(p_cam, R_nc):
    """Camera pose (camera->world) -> MonoGS world-to-camera (R, T): X_c = R X_w + T."""
    R = R_nc.T
    return R, -R @ np.asarray(p_cam, float)


def cam_pose_from_w2c(R, T):
    R_nc = np.asarray(R, float).T
    return -R_nc @ np.asarray(T, float), R_nc
