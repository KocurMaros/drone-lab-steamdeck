#!/usr/bin/env python3
"""Generate the Gazebo demo world and drone model from sim/course/arena.yaml.

    python3 tools/gen_arena.py [--iris-src ~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf]

Writes:
  sim/worlds/dronelab_arena.world
  sim/models/dronelab_iris/{model.sdf, model.config}   (only when --iris-src is given)

Course coordinates are ENU (x east, y north). Gazebo here uses x = north, y = west
(that is what the ArduPilot Gazebo plugin assumes with gazeboXYZToNED = 180 deg about x),
so every pose is converted: gz_x = north, gz_y = -east, gz_yaw = enu_yaw - 90 deg.
"""
from __future__ import annotations

import argparse
import math
import os
import re

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def gz(x, y):
    return y, -x


def gz_yaw(enu_yaw_deg):
    return math.radians(enu_yaw_deg - 90.0)


def color_mat(rgb, emissive=0.35):
    r, g, b = rgb
    return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.2 0.2 0.2 1</specular><emissive>{r * emissive:.3f} {g * emissive:.3f} {b * emissive:.3f} 1"
            f"</emissive></material>")


def script_mat(name):
    return (f"<material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
            f"<name>{name}</name></script></material>")


def box_visual(name, pose, size, material, collision=False):
    px, py, pz, rr, rp, ry = pose
    sx, sy, sz = size
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    out = f"""
        <visual name="{name}_v"><pose>{px:.3f} {py:.3f} {pz:.3f} {rr:.4f} {rp:.4f} {ry:.4f}</pose>{geo}{material}</visual>"""
    if collision:
        out += f"""
        <collision name="{name}_c"><pose>{px:.3f} {py:.3f} {pz:.3f} {rr:.4f} {rp:.4f} {ry:.4f}</pose>{geo}</collision>"""
    return out


def gate_model(i, g, size):
    W, H, bar = size["width"], size["height"], size["bar"]
    gx, gy = gz(g["x"], g["y"])
    yaw = gz_yaw(g["yaw"])
    z = g["z"]
    mat = color_mat(g["color"])
    # frame in the model's y-z plane (the drone flies along the model's x axis)
    parts = [
        box_visual("top", (0, 0, H / 2 + bar / 2, 0, 0, 0), (bar, W + 2 * bar, bar), mat),
        box_visual("bottom", (0, 0, -H / 2 - bar / 2, 0, 0, 0), (bar, W + 2 * bar, bar), mat),
        box_visual("left", (0, W / 2 + bar / 2, 0, 0, 0, 0), (bar, bar, H), mat),
        box_visual("right", (0, -W / 2 - bar / 2, 0, 0, 0, 0), (bar, bar, H), mat),
    ]
    leg_h = z - H / 2 - bar
    if leg_h > 0.05:
        for side, yy in (("legl", W / 2 + bar / 2), ("legr", -W / 2 - bar / 2)):
            parts.append(box_visual(side, (0, yy, -H / 2 - bar - leg_h / 2, 0, 0, 0), (0.08, 0.08, leg_h),
                                    script_mat("Gazebo/DarkGrey")))
    # a big translucent number plate is not possible in classic Gazebo without textures, so the
    # gate order is shown by colour + the Demo app's HUD/minimap.
    return f"""
    <model name="gate_{i + 1}">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def building_model(i, b):
    gx, gy = gz(b["x"], b["y"])
    # w = east size, d = north size -> gazebo x = north (d), y = west (w)
    size = (b["d"], b["w"], b["h"])
    return f"""
    <model name="building_{i + 1}">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} {b['h'] / 2:.3f} 0 0 0</pose>
      <link name="link">{box_visual('body', (0, 0, 0, 0, 0, 0), size, script_mat(b.get('material', 'Gazebo/Grey')), True)}
        {box_visual('roof', (0, 0, b['h'] / 2 + 0.1, 0, 0, 0), (size[0] + 0.4, size[1] + 0.4, 0.2), script_mat('Gazebo/DarkGrey'), True)}
      </link>
    </model>"""


def cyl(name, pose, r, h, material, collision=True):
    px, py, pz = pose
    geo = f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"
    s = f"""
        <visual name="{name}_v"><pose>{px} {py} {pz} 0 0 0</pose>{geo}{material}</visual>"""
    if collision:
        s += f"""
        <collision name="{name}_c"><pose>{px} {py} {pz} 0 0 0</pose>{geo}</collision>"""
    return s


def tree_model(i, t):
    gx, gy = gz(t[0], t[1])
    trunk = cyl("trunk", (0, 0, 1.25), 0.25, 2.5, color_mat((0.35, 0.22, 0.12), 0.0))
    crown_geo = "<geometry><sphere><radius>2.0</radius></sphere></geometry>"
    crown_mat = color_mat((0.13, 0.45, 0.16), 0.05)
    return f"""
    <model name="tree_{i + 1}">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} 0 0 0 0</pose>
      <link name="link">{trunk}
        <visual name="crown_v"><pose>0 0 3.8 0 0 0</pose>{crown_geo}{crown_mat}</visual>
        <collision name="crown_c"><pose>0 0 3.8 0 0 0</pose>{crown_geo}</collision>
        <visual name="crown2_v"><pose>0.4 0.3 5.0 0 0 0</pose><geometry><sphere><radius>1.4</radius></sphere></geometry>{crown_mat}</visual>
      </link>
    </model>"""


def tower_model(i, t):
    gx, gy = gz(t["x"], t["y"])
    h = t["h"]
    body = cyl("mast", (0, 0, h / 2), t["r"], h, script_mat("Gazebo/White"))
    stripes = "".join(cyl(f"stripe{k}", (0, 0, h * (k + 0.5) / 6), t["r"] + 0.02, h / 12, script_mat("Gazebo/Red"), False)
                      for k in range(6))
    light = ("<visual name='lamp'><pose>0 0 %.2f 0 0 0</pose><geometry><sphere><radius>0.35</radius></sphere>"
             "</geometry>%s</visual>" % (h + 0.3, script_mat("Gazebo/RedGlow")))
    return f"""
    <model name="tower_{i + 1}">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} 0 0 0 0</pose>
      <link name="link">{body}{stripes}
        {light}
      </link>
    </model>"""


def pad_model(p):
    gx, gy = gz(p["x"], p["y"])
    white = script_mat("Gazebo/White")
    parts = [
        cyl("pad", (0, 0, 0.01), 2.0, 0.02, script_mat("Gazebo/DarkGrey"), False),
        cyl("ring", (0, 0, 0.012), 1.75, 0.02, script_mat("Gazebo/Yellow"), False),
        cyl("inner", (0, 0, 0.014), 1.6, 0.02, script_mat("Gazebo/DarkGrey"), False),
        # the H (drone faces north = gazebo +x, so the H bars run along x)
        box_visual("h_l", (0, 0.45, 0.026, 0, 0, 0), (1.4, 0.22, 0.01), white),
        box_visual("h_r", (0, -0.45, 0.026, 0, 0, 0), (1.4, 0.22, 0.01), white),
        box_visual("h_m", (0, 0, 0.026, 0, 0, 0), (0.22, 0.9, 0.01), white),
    ]
    return f"""
    <model name="landing_pad">
      <static>true</static>
      <pose>{gx:.3f} {gy:.3f} 0 0 0 0</pose>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def fence_model(f):
    (x0, x1), (y0, y1) = f["x"], f["y"]
    zmax = f["z"][1]
    # corners in ENU -> gazebo
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    parts = []
    red = script_mat("Gazebo/Red")
    tape = script_mat("Gazebo/RedTransparent")
    for k, (a, b) in enumerate(zip(corners, corners[1:] + corners[:1])):
        ax, ay = gz(*a)
        bx, by = gz(*b)
        L = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        mx, my = (ax + bx) / 2, (ay + by) / 2
        parts.append(box_visual(f"tape{k}", (mx, my, 0.02, 0, 0, yaw), (L, 0.3, 0.02), tape))
        # posts every ~8 m with a red top
        n = max(2, int(L // 8))
        for j in range(n):
            t = j / n
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            parts.append(box_visual(f"post{k}_{j}", (px, py, 1.0, 0, 0, 0), (0.12, 0.12, 2.0), red))
    # corner masts show the ceiling
    for k, (cx, cy) in enumerate(corners):
        gx, gy = gz(cx, cy)
        parts.append(cyl(f"mast{k}", (gx, gy, zmax / 2), 0.08, zmax, script_mat("Gazebo/RedTransparent"), False))
    return f"""
    <model name="fence_markers">
      <static>true</static>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def roads(items):
    out = []
    for k, r in enumerate(items):
        gx, gy = gz(r["x"], r["y"])
        out.append(f"""        <visual name="road{k}">
          <pose>{gx:.2f} {gy:.2f} 0.005 0 0 0</pose>
          <geometry><box><size>{r['d']} {r['w']} 0.01</size></box></geometry>
          {script_mat('Gazebo/Road')}
        </visual>""")
    return "\n".join(out)


def world(course):
    home = course["home"]
    gates = "".join(gate_model(i, g, course["gate_size"]) for i, g in enumerate(course["gates"]))
    buildings = "".join(building_model(i, b) for i, b in enumerate(course.get("buildings", [])))
    trees = "".join(tree_model(i, t) for i, t in enumerate(course.get("trees", [])))
    towers = "".join(tower_model(i, t) for i, t in enumerate(course.get("towers", [])))
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_arena.py from sim/course/arena.yaml - edit the YAML, not this file. -->
<sdf version="1.6">
  <world name="dronelab_arena">
    <physics type="ode">
      <ode>
        <solver><type>quick</type><iters>100</iters><sor>1.0</sor></solver>
        <constraints><cfm>0.0</cfm><erp>0.2</erp><contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer></constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>-1</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{home['lat']}</latitude_deg>
      <longitude_deg>{home['lon']}</longitude_deg>
      <elevation>{home['alt_amsl']}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene>
      <ambient>0.55 0.55 0.6 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <sky><clouds><speed>3</speed></clouds></sky>
      <shadows>false</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-8 4 4 0 0.3 -0.4</pose>
        <track_visual>
          <name>dronelab_iris</name>
          <static>false</static>
          <use_model_frame>true</use_model_frame>
          <xyz>-5.5 0 2.2</xyz>
          <inherit_yaw>true</inherit_yaw>
          <min_dist>2.0</min_dist>
          <max_dist>15.0</max_dist>
        </track_visual>
      </camera>
    </gui>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>2.0</update_rate>
    </plugin>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="grass">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          {script_mat('Gazebo/Grass')}
        </visual>
{roads(course.get('roads', []))}
      </link>
    </model>
{pad_model(course['pad'])}
{fence_model(course['fence'])}
{gates}
{buildings}
{trees}
{towers}
    <include>
      <uri>model://dronelab_iris</uri>
      <name>dronelab_iris</name>
      <pose>{gz(course['pad']['x'], course['pad']['y'])[0]} {gz(course['pad']['x'], course['pad']['y'])[1]} 0.02 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


CAMERA = """
    <!-- FPV camera (added by tools/gen_arena.py) -->
    <link name="fpv_camera_link">
      <pose>0.26 0 0.21 0 0.26 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz></inertia>
      </inertial>
      <visual name="fpv_camera_visual">
        <geometry><box><size>0.04 0.05 0.04</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Black</name></script></material>
      </visual>
      <sensor name="fpv" type="camera">
        <always_on>true</always_on>
        <update_rate>{fps}</update_rate>
        <visualize>false</visualize>
        <camera name="fpv">
          <horizontal_fov>1.75</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.12</near><far>400</far></clip>
        </camera>
        <plugin name="fpv_camera_ros" filename="libgazebo_ros_camera.so">
          <ros><namespace>/drone1</namespace></ros>
          <camera_name>camera</camera_name>
          <frame_name>fpv_camera_optical</frame_name>
        </plugin>
      </sensor>
    </link>
    <joint name="fpv_camera_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>fpv_camera_link</child>
    </joint>
"""


def iris_model(src_path, fps=30, w=640, h=480):
    sdf = open(src_path, encoding="utf-8").read()
    # drop the gimbal (it needs an online model) and rename the model
    sdf = re.sub(r"\s*<include>\s*<uri>model://gimbal_small_2d</uri>.*?</include>", "", sdf, flags=re.S)
    sdf = re.sub(r"\s*<joint name=\"iris_gimbal_mount\".*?</joint>", "", sdf, flags=re.S)
    sdf = re.sub(r"<!--.*?-->", "", sdf, flags=re.S)  # also removes the debug cp markers
    sdf = sdf.replace('<model name="iris_demo">', '<model name="dronelab_iris">')
    sdf = sdf.replace("iris_demo::iris::iris/imu_link::imu_sensor", "dronelab_iris::iris::iris/imu_link::imu_sensor")
    sdf = sdf.replace("</model>\n</sdf>", CAMERA.format(fps=fps, w=w, h=h) + "  </model>\n</sdf>")
    sdf = re.sub(r"\n\s*\n+", "\n", sdf)
    header = ("<?xml version='1.0'?>\n<!-- GENERATED by tools/gen_arena.py from ardupilot_gazebo "
              "(khancyr, GPL-3.0) iris_with_ardupilot + FPV camera. -->\n")
    return header + sdf.split("?>", 1)[1].lstrip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--course", default=os.path.join(ROOT, "sim", "course", "arena.yaml"))
    ap.add_argument("--iris-src", default=None)
    a = ap.parse_args()
    course = yaml.safe_load(open(a.course, encoding="utf-8"))
    out = os.path.join(ROOT, "sim", "worlds", "dronelab_arena.world")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w", encoding="utf-8").write(world(course))
    print("wrote", out)
    if a.iris_src:
        mdir = os.path.join(ROOT, "sim", "models", "dronelab_iris")
        os.makedirs(mdir, exist_ok=True)
        open(os.path.join(mdir, "model.sdf"), "w", encoding="utf-8").write(iris_model(a.iris_src))
        open(os.path.join(mdir, "model.config"), "w", encoding="utf-8").write(
            "<?xml version='1.0'?>\n<model>\n  <name>DroneLab Iris (ArduPilot + FPV camera)</name>\n"
            "  <version>1.0</version>\n  <sdf version='1.6'>model.sdf</sdf>\n"
            "  <description>iris_with_ardupilot without gimbal, with a ROS 2 FPV camera.</description>\n</model>\n")
        print("wrote", mdir)


if __name__ == "__main__":
    main()
