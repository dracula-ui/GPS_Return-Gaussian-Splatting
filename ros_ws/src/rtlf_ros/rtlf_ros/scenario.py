"""Scenario geometry shared by the world generator, the mission logic and the tests.
All coordinates are PX4 local NED metres (n = north, e = east) relative to the take-off point.
In Gazebo (ENU world, vehicle spawned at the origin) gz_x = e and gz_y = n."""

ALT = 15.0            # cruise altitude [m]; all buildings are taller than this
CRUISE = 5.0          # default cruise speed for the emulated estimator [m/s]
CRUISE_SLAM = 3.0     # default cruise speed when a real SLAM provides the pose [m/s]

# Outbound route. The first leg weaves (+-12 m) so the SLAM frame is well conditioned and keyframes
# see the scene from different angles. GNSS starts degrading when ROUTE[DENIAL_WP] is reached.
ROUTE = [(0.0, 0.0), (12.0, 50.0), (-12.0, 100.0), (0.0, 150.0), (90.0, 150.0), (90.0, 60.0), (90.0, -20.0)]
DENIAL_WP = 3

# Buildings: (n_centre, e_centre, size_n, size_e, height)
BUILDINGS = [
    (45.0, 70.0, 30.0, 30.0, 30.0),   # on the straight line back to the anchor
    (60.0, 130.0, 20.0, 20.0, 30.0),
    (20.0, 10.0, 15.0, 15.0, 30.0),
]

# Low-texture patches (n_min, n_max, e_min, e_max): rendered as a flat uniform slab in the world.
SPARSE = [(20.0, 70.0, 100.0, 170.0)]

# Planner grid bounds (n_min, n_max, e_min, e_max)
BOUNDS = (-40.0, 140.0, -60.0, 200.0)


def in_building(n, e, margin=0.0):
    for (cn, ce, sn, se, _h) in BUILDINGS:
        if abs(n - cn) <= sn / 2 + margin and abs(e - ce) <= se / 2 + margin:
            return True
    return False


def in_sparse(n, e):
    return any(n0 <= n <= n1 and e0 <= e <= e1 for (n0, n1, e0, e1) in SPARSE)
