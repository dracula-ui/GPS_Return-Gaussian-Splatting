#!/usr/bin/env python3
"""Generate the Gazebo world, textures and the RGB-D drone model into sim/generated/.

    python3 make_world.py            # (re)generate everything
Options: --rtf 1.0 (real-time factor cap), --ground mesh|plane, --seed 7

Why textures: SLAM needs visual structure. PX4's default world has a plain grey ground and would make
photometric tracking fail, so the ground and the buildings get procedural multi-scale textures. The
low-texture patch in scenario.SPARSE is a flat uniform slab on top of the textured ground.
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros_ws", "src", "rtlf_ros"))
from rtlf_ros import scenario as S           # noqa: E402
from rtlf_ros import sim_params as P         # noqa: E402


def pink_noise(n, rng, beta=1.4):
    """Seamlessly tileable 1/f^beta noise in [0, 1]."""
    fy = np.fft.fftfreq(n)[:, None]; fx = np.fft.fftfreq(n)[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2); f[0, 0] = 1.0
    spec = (rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n))) / f ** beta
    spec[0, 0] = 0
    a = np.real(np.fft.ifft2(spec))
    a = (a - a.min()) / (a.max() - a.min() + 1e-12)
    return a


def stretch(a, lo=2, hi=98):
    l, h = np.percentile(a, [lo, hi])
    return np.clip((a - l) / (h - l + 1e-12), 0, 1)


def speckles(n, rng, count, radius):
    """Random soft dots (tileable by wrap-around) for local, distinctive features."""
    img = np.zeros((n, n)); R = int(6 * radius) + 1
    ax = np.arange(-R, R + 1); gy, gx = np.meshgrid(ax, ax, indexing="ij")
    for _ in range(count):
        cy, cx = rng.integers(0, n, 2); r = radius * (0.5 + rng.random()); sign = 1 if rng.random() < 0.5 else -1
        patch = sign * np.exp(-(gx ** 2 + gy ** 2) / (2 * r ** 2)) * (0.5 + rng.random())
        img[np.ix_((cy + ax) % n, (cx + ax) % n)] += patch
    return img


def make_ground_texture_irregular(path, rng, n=1024):
    from PIL import Image
    r = stretch(pink_noise(n, rng, 1.0)); g = stretch(0.6 * pink_noise(n, rng, 1.0) + 0.4 * r); b = stretch(0.5 * pink_noise(n, rng, 1.0) + 0.3 * r)
    base = np.stack([0.30 + 0.50 * r, 0.35 + 0.45 * g, 0.18 + 0.40 * b], axis=-1)
    fine = stretch(pink_noise(n, rng, beta=0.5))[..., None]         # high-frequency grain
    dots = speckles(n, rng, 900, 3.5)[..., None] * 0.18
    # mid-scale, high-contrast, non-repeating patches (0.3-1.5 m on a 20 m tile): the scale a 320x240 camera resolves from 10-30 m
    patches = speckles(n, rng, 260, 16.0)[..., None]
    patches = patches / (np.abs(patches).max() + 1e-9)
    img = np.clip(base * (0.65 + 0.7 * fine) + dots + 0.45 * patches, 0, 1)
    Image.fromarray((img * 255).astype(np.uint8)).save(path)


def make_building_texture_irregular(path, rng, n=1024):
    """Facade with irregular windows (random size, position, brightness), panel seams and stains: no periodic grid."""
    from PIL import Image
    base = 0.25 + 0.55 * stretch(pink_noise(n, rng, 0.9)) + 0.1 * speckles(n, rng, 600, 3.0)
    base = base + 0.35 * speckles(n, rng, 120, 22.0) / 1.5          # stains / patches of different brightness
    img = np.stack([base, base * 0.95, base * 0.9], axis=-1)
    for _ in range(70):                                            # irregular windows
        w = int(rng.integers(n // 40, n // 9)); h = int(rng.integers(n // 40, n // 7))
        x0 = int(rng.integers(0, n - w)); y0 = int(rng.integers(0, n - h))
        shade = 0.05 + 0.5 * rng.random(); tint = np.array([shade, shade * (0.9 + 0.3 * rng.random()), shade * (1.0 + 0.6 * rng.random())])
        img[y0:y0 + h, x0:x0 + w, :] = tint
        img[y0:y0 + 3, x0:x0 + w, :] = 0.8 * tint + 0.2                # light sill / frame line
    for _ in range(14):                                            # panel seams
        if rng.random() < 0.5:
            y = int(rng.integers(0, n)); img[y:y + 2, :, :] *= 0.45
        else:
            x = int(rng.integers(0, n)); img[:, x:x + 2, :] *= 0.45
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def make_ground_texture_classic(path, rng, n=1024):
    from PIL import Image
    r = stretch(pink_noise(n, rng, 1.0)); g = stretch(0.6 * pink_noise(n, rng, 1.0) + 0.4 * r); b = stretch(0.5 * pink_noise(n, rng, 1.0) + 0.3 * r)
    base = np.stack([0.30 + 0.50 * r, 0.35 + 0.45 * g, 0.18 + 0.40 * b], axis=-1)
    fine = stretch(pink_noise(n, rng, beta=0.5))[..., None]         # high-frequency grain
    dots = speckles(n, rng, 900, 3.5)[..., None] * 0.18
    img = np.clip(base * (0.65 + 0.7 * fine) + dots, 0, 1)
    Image.fromarray((img * 255).astype(np.uint8)).save(path)


def make_building_texture_classic(path, rng, n=1024):
    from PIL import Image
    base = 0.25 + 0.55 * stretch(pink_noise(n, rng, 0.9)) + 0.1 * speckles(n, rng, 600, 3.0)
    img = np.stack([base, base * 0.95, base * 0.9], axis=-1)
    cell = n // 8
    for i in range(1, 8, 1):                                       # window grid
        for j in range(1, 8, 1):
            y0, x0 = i * cell - cell // 3, j * cell - cell // 3
            shade = 0.15 + 0.25 * rng.random()
            img[y0:y0 + cell // 2, x0:x0 + cell // 2, :] = np.array([shade, shade * 1.1, shade * 1.4])
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def make_ground_texture(path, rng, n=1024):
    (make_ground_texture_irregular if os.environ.get("TEXTURES") == "irregular" else make_ground_texture_classic)(path, rng, n)


def make_building_texture(path, rng, n=1024):
    (make_building_texture_irregular if os.environ.get("TEXTURES") == "irregular" else make_building_texture_classic)(path, rng, n)


def write_ground_obj(dirpath, size=700.0, tile=20.0):
    os.makedirs(os.path.join(dirpath, "meshes"), exist_ok=True)
    h = size / 2; t = size / tile
    with open(os.path.join(dirpath, "meshes", "ground.obj"), "w") as f:
        f.write("mtllib ground.mtl\nusemtl ground\n")
        f.write("v %f %f 0\nv %f %f 0\nv %f %f 0\nv %f %f 0\n" % (-h, -h, h, -h, h, h, -h, h))
        f.write("vt 0 0\nvt %f 0\nvt %f %f\nvt 0 %f\n" % (t, t, t, t))
        f.write("vn 0 0 1\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
    with open(os.path.join(dirpath, "meshes", "ground.mtl"), "w") as f:
        f.write("newmtl ground\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ../materials/textures/ground.png\n")


def building(i, n, e, sn, se, h):
    uri = "model://rtlf_textures/materials/textures/building.png"
    return f"""
    <model name="bldg_{i}"><static>true</static><pose>{e} {n} {h/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{se} {sn} {h}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{se} {sn} {h}</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal><albedo_map>{uri}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual>
      </link></model>"""


def clutter_boxes(count, seed=11):
    """Low static boxes (3-10 m tall, well below the 15 m flight altitude) 10-35 m beside the route: real 3D relief for the
    camera instead of a flat ground plane. Off the route and below the flight altitude, so the planner map and collision checks are unchanged."""
    rng = np.random.default_rng(seed)
    route = np.array(S.ROUTE, float)

    def dist_route(n, e):
        d = 1e9
        for a, b in zip(route[:-1], route[1:]):
            ab = b - a; t = np.clip(np.dot([n, e] - a, ab) / np.dot(ab, ab), 0, 1)
            d = min(d, float(np.linalg.norm(np.array([n, e]) - (a + t * ab))))
        return d
    out = []
    tries = 0
    while len(out) < count and tries < 20000:
        tries += 1
        n = rng.uniform(S.BOUNDS[0] + 10, S.BOUNDS[1] - 10); e = rng.uniform(S.BOUNDS[2] + 10, S.BOUNDS[3] - 10)
        sn, se = rng.uniform(3, 8, 2); h = rng.uniform(3, 10)
        if not (10.0 <= dist_route(n, e) <= 35.0):
            continue
        if S.in_building(n, e, margin=8.0) or S.in_sparse(n, e):
            continue
        if any(abs(n - o[0]) < (sn + o[2]) / 2 + 3 and abs(e - o[1]) < (se + o[3]) / 2 + 3 for o in out):
            continue
        out.append((n, e, sn, se, h))
    return out


def clutter_sdf():
    count = int(os.environ.get("CLUTTER", "0"))
    return "".join(building("c%d" % i, *x) for i, x in enumerate(clutter_boxes(count))) if count > 0 else ""


def sparse(i, n0, n1, e0, e1):
    return f"""
    <model name="sparse_{i}"><static>true</static><pose>{(e0+e1)/2} {(n0+n1)/2} 0.02 0 0 0</pose>
      <link name="link">
        <visual name="v"><geometry><box><size>{e1-e0} {n1-n0} 0.02</size></box></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse>
          <specular>0 0 0 1</specular></material></visual>
      </link></model>"""


def ground_visual(mode):
    if mode == "mesh":
        return """<visual name="visual"><geometry><mesh><uri>model://rtlf_ground/meshes/ground.obj</uri></mesh></geometry></visual>"""
    uri = "model://rtlf_textures/materials/textures/ground.png"
    return f"""<visual name="visual"><geometry><plane><normal>0 0 1</normal><size>700 700</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal><albedo_map>{uri}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual>"""


def world_sdf(rtf, ground):
    b = "".join(building(i, *x) for i, x in enumerate(S.BUILDINGS))
    z = "".join(sparse(i, *x) for i, x in enumerate(S.SPARSE)) + clutter_sdf()
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{P.WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
      <real_time_update_rate>{int(250 * rtf) if rtf < 1 else 250}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.74 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="collision"><pose>0 0 -5 0 0 0</pose>
          <geometry><box><size>100000 100000 10</size></box></geometry></collision>
        {ground_visual(ground)}
      </link>
    </model>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows><intensity>1.0</intensity>
      <direction>0.3 0.3 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg><longitude_deg>8.546163739800146</longitude_deg><elevation>488</elevation>
    </spherical_coordinates>
{b}
{z}
    <include>
      <uri>model://{P.MODEL_URI}</uri>
      <name>{P.MODEL_NAME}</name>
      <pose>0 0 0.25 0 0 1.5708</pose>
    </include>
  </world>
</sdf>
"""


def vehicle_model():
    fwd, right, down = P.LEVER_FRD
    flu = (fwd, -right, -down)          # gz link frame is FLU
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{P.MODEL_URI}">
    <include merge="true"><uri>model://x500</uri></include>
    <link name="rgbd_link">
      <pose relative_to="base_link">{flu[0]} {flu[1]} {flu[2]} 0 {math.radians(P.CAM_PITCH_DEG):.6f} 0</pose>
      <inertial><mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
      <visual name="rgbd_visual"><geometry><box><size>0.02 0.06 0.02</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      <sensor name="rgbd" type="rgbd_camera">
        <camera name="rgbd">
          <horizontal_fov>{P.HFOV}</horizontal_fov>
          <image><width>{P.WIDTH}</width><height>{P.HEIGHT}</height></image>
          <clip><near>{P.NEAR}</near><far>{P.FAR}</far></clip>
          <depth_camera><clip><near>{P.NEAR}</near><far>{P.FAR}</far></clip></depth_camera>
        </camera>
        <always_on>1</always_on>
        <update_rate>{P.RATE_HZ}</update_rate>
        <visualize>false</visualize>
        <topic>{P.SENSOR_TOPIC}</topic>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on><update_rate>100</update_rate><visualize>false</visualize>
        <topic>/world/{P.WORLD_NAME}/model/{P.MODEL_NAME}/link/base_link/sensor/magnetometer_sensor/magnetometer</topic>
        <magnetometer>
          <x><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise></z>
        </magnetometer>
      </sensor>
    </link>
    <joint name="rgbd_joint" type="fixed"><parent>base_link</parent><child>rgbd_link</child></joint>
  </model>
</sdf>
"""


MODEL_CONFIG = """<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>{desc}</description></model>
"""

SERVER_CONFIG = """<server_config>
  <plugins>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </plugins>
</server_config>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtf", type=float, default=1.0)
    ap.add_argument("--ground", choices=["mesh", "plane"], default="mesh")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default=os.path.join(HERE, "generated"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    out = a.out
    models = os.path.join(out, "models"); worlds = os.path.join(out, "worlds")
    tex = os.path.join(models, "rtlf_textures", "materials", "textures")
    for d in (models, worlds, tex, os.path.join(models, "rtlf_ground"), os.path.join(models, P.MODEL_URI),
              os.path.join(models, "rtlf_ground", "materials", "textures")):
        os.makedirs(d, exist_ok=True)
    make_ground_texture(os.path.join(tex, "ground.png"), rng)
    make_building_texture(os.path.join(tex, "building.png"), rng)
    make_ground_texture(os.path.join(models, "rtlf_ground", "materials", "textures", "ground.png"), np.random.default_rng(a.seed))
    write_ground_obj(os.path.join(models, "rtlf_ground"))
    for name, desc in (("rtlf_textures", "textures"), ("rtlf_ground", "textured ground mesh")):
        open(os.path.join(models, name, "model.config"), "w").write(MODEL_CONFIG.format(name=name, desc=desc))
        if name == "rtlf_textures":
            open(os.path.join(models, name, "model.sdf"), "w").write('<?xml version="1.0"?><sdf version="1.9"><model name="rtlf_textures"><static>true</static></model></sdf>')
    open(os.path.join(models, "rtlf_ground", "model.sdf"), "w").write('<?xml version="1.0"?><sdf version="1.9"><model name="rtlf_ground"><static>true</static></model></sdf>')
    open(os.path.join(models, P.MODEL_URI, "model.config"), "w").write(MODEL_CONFIG.format(name=P.MODEL_URI, desc="x500 with forward RGB-D camera"))
    open(os.path.join(models, P.MODEL_URI, "model.sdf"), "w").write(vehicle_model())
    open(os.path.join(worlds, P.WORLD_NAME + ".sdf"), "w").write(world_sdf(a.rtf, a.ground))
    open(os.path.join(out, "server.config"), "w").write(SERVER_CONFIG)
    print("generated:", out)
    print("  world :", os.path.join(worlds, P.WORLD_NAME + ".sdf"))
    print("  camera: %dx%d hfov=%.2f rad fx=%.2f cx=%.1f cy=%.1f" % (P.WIDTH, P.HEIGHT, P.HFOV, P.FX, P.CX, P.CY))


if __name__ == "__main__":
    main()
