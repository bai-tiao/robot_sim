#!/usr/bin/env python3
"""
isaac_scene.py — Isaac Sim 仿真场景
=====================================
自动从 robot_description 包导入 URDF，添加 RTX LiDAR，
发布 /scan /odom /cmd_vel 供 Nav2 使用。
"""

import os, sys

os.environ.setdefault("VK_ICD_FILENAMES", "/usr/share/vulkan/icd.d/nvidia_icd.json")
os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")


def _find_offline_kit():
    env_kit = os.environ.get("ISAAC_OFFLINE_KIT", "")
    if env_kit and os.path.exists(env_kit):
        return env_kit
    for p in sys.path:
        candidate = os.path.join(p, "isaacsim", "apps", "isaacsim.exp.base.python.offline.kit")
        if os.path.exists(candidate):
            return candidate
    return ""


def _find_xacro_path():
    """只定位 xacro 文件路径，不做任何 import（必须在 SimulationApp 之前调用安全）"""
    xacro_path = os.environ.get("ROBOT_URDF_XACRO", "")
    if not xacro_path:
        ws_root = os.environ.get("ROBOT_WS", "")
        if not ws_root:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            ws_root = os.path.normpath(os.path.join(script_dir, "..", "..", ".."))
        xacro_path = os.path.join(
            ws_root, "src", "robot_description", "urdf", "robot.urdf.xacro"
        )
    print(f"[isaac_scene] xacro 路径: {xacro_path}", flush=True)
    if not os.path.exists(xacro_path):
        print(f"[isaac_scene] ⚠️  找不到 xacro: {xacro_path}", flush=True)
        return ""
    return xacro_path


def _expand_urdf(xacro_path):
    """展开 xacro → 临时 URDF 文件，必须在 SimulationApp 创建之后调用"""
    if not xacro_path:
        return ""
    import tempfile

    # 优先 subprocess（隔离 Python 环境，避免污染 omni 模块空间）
    import subprocess
    for ros2_bin in ["/opt/ros/humble/bin/ros2", "ros2"]:
        out = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False)
        ret = subprocess.run(
            [ros2_bin, "run", "xacro", "xacro", xacro_path],
            stdout=out, stderr=subprocess.PIPE,
            env={**os.environ, "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages"}
        )
        out.close()
        if ret.returncode == 0 and os.path.getsize(out.name) > 100:
            print(f"[isaac_scene] URDF (subprocess): {out.name}", flush=True)
            return out.name

    # 降级：在当前进程内用 xacro 模块（SimulationApp 已启动，风险较低）
    try:
        import xacro
        doc = xacro.process_file(xacro_path)
        out = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
        out.write(doc.toprettyxml(indent="  "))
        out.close()
        print(f"[isaac_scene] URDF (xacro module): {out.name}", flush=True)
        return out.name
    except Exception as e:
        print(f"[isaac_scene] ⚠️  xacro module 失败: {e}", flush=True)

    print("[isaac_scene] ⚠️  URDF 展开失败", flush=True)
    return ""


_OFFLINE_KIT  = _find_offline_kit()
_HEADLESS     = os.environ.get("ISAAC_HEADLESS", "0") == "1"
_XACRO_PATH   = _find_xacro_path()   # 只找路径，不 import xacro

print(f"[isaac_scene] offline_kit={_OFFLINE_KIT or '(默认)'}", flush=True)

from isaacsim import SimulationApp
app = SimulationApp({
    "headless": _HEADLESS,
    # Storm = OpenGL/Hydra 渲染器，不依赖 RTX，对 50 系显卡兼容性最好
    # RayTracedLighting = RTX 渲染器（50 系有兼容性问题，会导致线框/黑屏）
    "renderer": "Storm",
    "width": 1280,
    "height": 720,
    "anti_aliasing": 0,
    "experience": _OFFLINE_KIT,
})
print("[isaac_scene] SimulationApp ready (renderer=Storm/OpenGL)", flush=True)

import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
import carb

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("omni.isaac.range_sensor")   # PhysX LiDAR 命令 + OmniGraph 节点
app.update()
print("[isaac_scene] extensions enabled", flush=True)


def load_robot(world):
    """展开 xacro 并导入 URDF（在 SimulationApp 启动后调用）"""
    urdf_path = _expand_urdf(_XACRO_PATH)
    if not urdf_path:
        print("[isaac_scene] ⚠️  跳过 URDF 导入（文件未找到）", flush=True)
        return False

    import omni.kit.commands

    # Isaac Sim 4.5 正确用法：先创建 ImportConfig，再导入
    ret, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    if not ret:
        print("[isaac_scene] ❌ 无法创建 ImportConfig", flush=True)
        return False

    cfg.merge_fixed_joints             = False
    cfg.convex_decomp                  = False
    cfg.import_inertia_tensor          = True
    cfg.fix_base                       = False   # 差速底盘，不固定根节点
    cfg.make_default_prim              = True
    cfg.self_collision                 = False
    # ★ False 必须：World() 已创建 PhysicsScene，再创建第二个会导致机器人和墙壁处于不同物理场景 → 穿墙
    cfg.create_physics_scene           = False
    cfg.default_drive_strength         = 1047.19751
    cfg.default_position_drive_damping = 52.35988
    # 设置默认驱动类型为速度驱动（1=位置, 2=速度）
    try:
        cfg.default_drive_type = 2
    except Exception:
        pass  # 老版本属性名可能不同

    # Isaac 4.5 不支持 dest_path 参数，直接不带
    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=cfg,
    )
    if result:
        print(f"[isaac_scene] ✅ URDF 导入成功 → {prim_path}", flush=True)
        return prim_path   # 返回实际 prim 路径
    else:
        print("[isaac_scene] ❌ URDF 导入失败", flush=True)
        return None


def add_lidar(robot_ok, lidar_prim):
    """在 lidar_link 下创建 PhysX 旋转 LiDAR sensor（不需要渲染管线）"""
    if not robot_ok:
        return ""
    import omni.kit.commands, omni.usd
    from pxr import Gf

    stage = omni.usd.get_context().get_stage()

    # 确认 parent prim 存在；URDF importer 路径可能与预期不同
    parent = lidar_prim.rsplit("/", 1)[0]
    leaf   = lidar_prim.rsplit("/", 1)[-1]
    parent_p = stage.GetPrimAtPath(parent)
    print(f"[isaac_scene] LiDAR parent={parent} valid={parent_p and parent_p.IsValid()} "
          f"type={parent_p.GetTypeName() if parent_p and parent_p.IsValid() else 'N/A'}", flush=True)
    if not (parent_p and parent_p.IsValid()):
        # 搜索 lidar_link prim（排除 joint）
        print(f"[isaac_scene] ⚠️  parent {parent} 不存在，搜索 lidar_link...", flush=True)
        for p in stage.Traverse():
            if "lidar_link" in p.GetName().lower() and "joint" not in p.GetTypeName().lower():
                parent = str(p.GetPath())
                lidar_prim = f"{parent}/{leaf}"
                print(f"[isaac_scene]   找到 lidar_link prim: {parent} type={p.GetTypeName()}", flush=True)
                break
        else:
            # 打印所有 robot/lidar 相关 prim 用于诊断
            all_prims = [(str(p.GetPath()), p.GetTypeName()) for p in stage.Traverse()
                         if "diff_drive" in str(p.GetPath()).lower() or "lidar" in str(p.GetPath()).lower()]
            print("[isaac_scene] stage prim 列表 (lidar/robot):\n" +
                  "\n".join(f"  {path} [{typ}]" for path, typ in all_prims[:30]), flush=True)
            return ""

    try:
        success, sensor = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=leaf,
            parent=parent,
            min_range=0.1,
            max_range=100.0,          # URDF: max=100m
            draw_points=False,
            draw_lines=False,
            horizontal_fov=180.0,     # URDF: 前半圆 -90°~+90°
            vertical_fov=30.0,        # URDF: ±15° = 30° 总垂直 FOV
            horizontal_resolution=0.2, # URDF: 900点/180° = 0.2°
            vertical_resolution=2.0,  # URDF: 16线/30° ≈ 2°
            rotation_rate=10.0,       # URDF: 10Hz
            high_lod=False,
            yaw_offset=0.0,           # 朝前（与 URDF 关节方向一致）
        )
    except Exception as cmd_err:
        print(f"[isaac_scene] ⚠️  PhysX LiDAR 命令异常: {cmd_err}", flush=True)
        success = False
        sensor  = None
    if success:
        actual = f"{parent}/{leaf}"
        print(f"[isaac_scene] ✅ PhysX LiDAR 添加成功 → {actual}", flush=True)
        return actual
    else:
        print(f"[isaac_scene] ⚠️  PhysX LiDAR 添加失败（命令执行出错）", flush=True)
        return ""


def create_lidar_render_product(lidar_prim):
    """PhysX LiDAR 不需要 render product，此函数仅做路径验证"""
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        lp = stage.GetPrimAtPath(lidar_prim)
        if lp and lp.IsValid():
            print(f"[isaac_scene] PhysX LiDAR prim valid, type={lp.GetTypeName()} @ {lidar_prim}", flush=True)
            return lidar_prim   # 返回 lidar prim 路径本身（不是 render product）
        # 搜索任意 lidar prim
        for p in stage.Traverse():
            if "lidar" in p.GetName().lower():
                print(f"[isaac_scene]   找到 LiDAR prim: {p.GetPath()} type={p.GetTypeName()}", flush=True)
                return str(p.GetPath())
    except Exception as e:
        print(f"[isaac_scene] ⚠️  lidar prim 验证失败: {e}", flush=True)
    return ""


def build_ros2_graph(robot_root, render_product_path=""):
    keys = og.Controller.Keys

    # /clock — Nav2 use_sim_time 依赖此话题
    og.Controller.edit(
        {"graph_path": "/Graphs/Clock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick",    "omni.graph.action.OnPlaybackTick"),
                ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock",   "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick",              "clock.inputs:execIn"),
                ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
            ],
        },
    )
    print("[isaac_scene] ✅ ROS2 OmniGraph 就绪 | /clock", flush=True)

    # /lidar/points — PhysX LiDAR OmniGraph: IsaacReadLidarPointCloud → ROS2PublishPointCloud
    # 3D 半球形雷达 (16线 ±15°) 发布 PointCloud2，供 pointcloud_to_laserscan 转换为 /scan
    if render_product_path:   # 此处 render_product_path 实为 lidar prim 路径
        try:
            og.Controller.edit(
                {"graph_path": "/Graphs/LiDAR", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("tick",    "omni.graph.action.OnPlaybackTick"),
                        ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("read",    "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                        ("pub",     "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                    ],
                    keys.CONNECT: [
                        ("tick.outputs:tick",               "read.inputs:execIn"),
                        ("read.outputs:execOut",            "pub.inputs:execIn"),
                        ("simtime.outputs:simulationTime",  "pub.inputs:timeStamp"),
                        ("read.outputs:data",               "pub.inputs:data"),   # 新版 API: data (旧版: pointCloudData)
                    ],
                    keys.SET_VALUES: [
                        ("pub.inputs:topicName",  "/lidar/points"),
                        ("pub.inputs:frameId",    "lidar_link"),
                    ],
                },
            )
            # lidarPrim 是 target 类型，必须单独设置
            import omni.usd
            from pxr import Sdf
            read_node = og.Controller.node("/Graphs/LiDAR/read")
            if read_node.is_valid():
                target_attr = read_node.get_attribute("inputs:lidarPrim")
                if target_attr.is_valid():
                    target_attr.set([render_product_path])
                    print(f"[isaac_scene] lidarPrim target 设置成功: {render_product_path}", flush=True)
                else:
                    print(f"[isaac_scene] ⚠️  lidarPrim target 属性不存在", flush=True)
            print(f"[isaac_scene] ✅ PhysX LiDAR OmniGraph 就绪 | /lidar/points PointCloud2 (lidar={render_product_path})", flush=True)
        except Exception as e:
            print(f"[isaac_scene] ⚠️  LiDAR OmniGraph 失败: {e}", flush=True)
    else:
        print("[isaac_scene] ⚠️  无 lidar prim，跳过 /lidar/points OmniGraph", flush=True)


def _make_material(stage, mat_path, rgb=(0.8, 0.8, 0.8), roughness=0.6, metallic=0.0):
    """创建 UsdPreviewSurface 材质（Storm 渲染器下显示实体颜色）"""
    from pxr import UsdShade, Sdf, Gf
    material = UsdShade.Material.Define(stage, mat_path)
    shader   = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f ).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness",     Sdf.ValueTypeNames.Float   ).Set(roughness)
    shader.CreateInput("metallic",      Sdf.ValueTypeNames.Float   ).Set(metallic)
    shader.CreateInput("opacity",       Sdf.ValueTypeNames.Float   ).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return material


def add_simple_room(world, half_x=8.0, half_y=6.0, height=3.0, thickness=0.3):
    """
    用 Isaac FixedCuboid 生成四面墙。
    FixedCuboid 内部正确创建 visual mesh + PhysX 碰撞体，
    避免手动 UsdGeom.Cube+XformOp Scale 导致 PhysX 无法推导碰撞尺寸的问题。
    LiDAR 能扫到、小车不会穿墙、Storm 渲染器下显示实体。
    """
    import numpy as np
    from isaacsim.core.api.objects import FixedCuboid

    wall_color = np.array([0.82, 0.82, 0.82])   # 浅灰色

    walls = [
        # (name,    scale_xyz(宽/深/高),                          position_xyz)
        ("wall_N", (half_x*2+thickness*2, thickness,  height),  (0.0,  half_y + thickness/2,  height/2)),
        ("wall_S", (half_x*2+thickness*2, thickness,  height),  (0.0, -half_y - thickness/2,  height/2)),
        ("wall_E", (thickness,  half_y*2,              height),  ( half_x + thickness/2, 0.0,  height/2)),
        ("wall_W", (thickness,  half_y*2,              height),  (-half_x - thickness/2, 0.0,  height/2)),
    ]
    for name, scale, pos in walls:
        world.scene.add(FixedCuboid(
            prim_path=f"/World/Room/{name}",
            name=f"room_{name}",
            position=np.array(pos, dtype=float),
            scale=np.array(scale, dtype=float),
            color=wall_color,
        ))
    print(f"[isaac_scene] ✅ 房间生成 {half_x*2:.0f}×{half_y*2:.0f}m "
          f"（FixedCuboid × 4，含 PhysX 碰撞 + visual mesh）", flush=True)


def main():
    from isaacsim.core.api.objects import GroundPlane as IsaacGroundPlane
    import numpy as np

    world = World(stage_units_in_meters=1.0)

    # 地板（纯代码生成，不联网）
    world.scene.add(IsaacGroundPlane(
        prim_path="/World/GroundPlane",
        name="ground",
        size=100.0,
        color=np.array([0.3, 0.3, 0.3]),
    ))

    # 加载自定义场景（可选）；否则生成内置房间
    scene_usd = os.environ.get("ISAAC_SCENE_USD", "")
    if scene_usd and os.path.exists(scene_usd):
        add_reference_to_stage(usd_path=scene_usd, prim_path="/World/Environment")
        print(f"[isaac_scene] 场景: {scene_usd}", flush=True)
    else:
        # ★ 必须在 world.reset() 之前调用：FixedCuboid 需要在 reset() 前加入 scene
        add_simple_room(world)

    # 导入机器人 URDF
    imported_prim = load_robot(world)   # 返回 prim 路径字符串或 None
    robot_ok = imported_prim is not None

    # 确定机器人根 prim 路径
    robot_root = "/World/diff_drive_robot"
    lidar_prim = f"{robot_root}/lidar_link/lidar"
    if robot_ok:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        # 系统保留 prim 名称（不是机器人）
        _WORLD_SKIP = {"GroundPlane", "Graphs", "Physics_Materials", "Looks",
                       "physicsScene", "PhysicsScene", "Environment"}
        _ROOT_SKIP  = {"World", "Render", "OmniKit_Viewport_LightRig",
                       "physicsScene", "PhysicsScene", "Looks", "Physics_Materials"}
        found = ""
        # 如果 importer 返回了具体路径，直接用
        if imported_prim and imported_prim.startswith("/"):
            # 验证 prim 存在
            test = stage.GetPrimAtPath(imported_prim)
            if test.IsValid():
                found = imported_prim
        # 如果没拿到或路径无效，遍历搜索
        if not found:
            world_prim = stage.GetPrimAtPath("/World")
            for child in world_prim.GetChildren():
                if child.GetPath().name not in _WORLD_SKIP and child.IsValid():
                    found = str(child.GetPath())
                    break
        if not found:
            root_prim = stage.GetPseudoRoot()
            for child in root_prim.GetChildren():
                n = child.GetPath().name
                if n not in _ROOT_SKIP and not n.startswith("_") and child.IsValid():
                    found = str(child.GetPath())
                    break
        if found:
            robot_root = found
        print(f"[isaac_scene] 机器人根 prim: {robot_root}", flush=True)
    lidar_prim_path = f"{robot_root}/lidar_link/lidar"

    # ★ IsaacRobot 必须在 world.reset() 之前加入场景
    _robot_art_holder = [None]
    if robot_ok:
        try:
            from isaacsim.core.api.robots import Robot as IsaacRobot
            _robot_art_holder[0] = IsaacRobot(prim_path=robot_root)
            world.scene.add(_robot_art_holder[0])
            print(f"[isaac_scene] IsaacRobot 已加入场景（pre-reset）: {robot_root}", flush=True)
        except Exception as e:
            print(f"[isaac_scene] ⚠️  IsaacRobot 加入场景失败: {e}", flush=True)

    world.reset()

    # ★ PhysX LiDAR 必须在 reset() 之后、play() 之前创建
    _lidar_prim_path = [lidar_prim_path]
    if robot_ok:
        _actual = add_lidar(robot_ok, lidar_prim_path)
        if _actual:
            _lidar_prim_path[0] = _actual

    world.play()   # ← 启动仿真时钟，发布 /clock，否则 timestamp=0
    # 步进若干帧让 physics / stage 完全初始化
    for _ in range(10):
        app.update()

    # ★ 验证 lidar prim 并确认实际路径
    _render_product_path = ""
    if robot_ok:
        lidar_prim_path = _lidar_prim_path[0]   # 更新为 add_lidar 返回的实际路径
        _render_product_path = create_lidar_render_product(lidar_prim_path)
        for _ in range(3):
            app.update()

    # 构建 ROS2 图
    if robot_ok:
        try:
            build_ros2_graph(robot_root, _render_product_path)
        except Exception as e:
            print(f"[isaac_scene] ⚠️  OmniGraph 警告: {e}", flush=True)
    else:
        print("[isaac_scene] ⚠️  URDF 导入失败，跳过 OmniGraph（避免 segfault）", flush=True)

    # 车轮关节控制器（差速驱动）—— initialize() 在 play() 后调用
    _art_ctrl = [None]
    _wheel_idx = [None]   # [left_idx, right_idx]
    if robot_ok and _robot_art_holder[0] is not None:
        try:
            _robot_art = _robot_art_holder[0]
            _robot_art.initialize()
            _art_ctrl[0] = _robot_art.get_articulation_controller()
            # 按名字查找车轮关节索引，避免顺序假设
            dof_names = list(_robot_art.dof_names)
            print(f"[isaac_scene] DOF 列表: {dof_names}", flush=True)
            li = next((i for i, n in enumerate(dof_names) if "left_wheel"  in n), 0)
            ri = next((i for i, n in enumerate(dof_names) if "right_wheel" in n), 1)
            _wheel_idx[0] = [li, ri]
            print(f"[isaac_scene] ✅ ArticulationController 就绪 @ {robot_root} | L={li} R={ri}", flush=True)

            # ★ 确保车轮关节处于 velocity drive 模式，设置足够的 damping
            try:
                import omni.usd
                from pxr import UsdPhysics
                stage = omni.usd.get_context().get_stage()
                for jname in dof_names:
                    if "wheel" not in jname:
                        continue
                    # continuous 关节的 DriveAPI token 是 "angular"
                    jprim = stage.GetPrimAtPath(f"{robot_root}/{jname}")
                    if not jprim.IsValid():
                        # 有些 URDF 结构是 robot_root/chassis/joint_name
                        for p in stage.Traverse():
                            if p.GetName() == jname:
                                jprim = p
                                break
                    if not jprim.IsValid():
                        print(f"[isaac_scene] ⚠️  找不到关节 prim: {jname}", flush=True)
                        continue
                    drive = UsdPhysics.DriveAPI.Get(jprim, "angular")
                    if not drive:
                        drive = UsdPhysics.DriveAPI.Apply(jprim, "angular")
                        print(f"[isaac_scene] ⚠️  {jname} 没有 DriveAPI，已新建", flush=True)
                    drive.GetTypeAttr().Set("velocity")
                    drive.GetDampingAttr().Set(1e6)    # 高 damping → 速度跟踪
                    drive.GetStiffnessAttr().Set(0.0)  # 零 stiffness → 纯速度驱动
                    print(f"[isaac_scene] ✅ {jname}: velocity drive, damping=1e6", flush=True)
            except Exception as e2:
                print(f"[isaac_scene] ⚠️  DriveAPI 配置失败: {e2}", flush=True)
        except Exception as e:
            print(f"[isaac_scene] ⚠️  ArticulationController 失败: {e}", flush=True)

    # ★ 记录机器人初始世界位姿，作为 odom 坐标系原点
    # Isaac Sim 坐标系 Z-up 与 ROS 相同，但 URDF 导入后初始四元数可能包含意外旋转
    _odom_origin = [None]   # (pos_x, pos_y, cos_yaw, sin_yaw)
    if robot_ok and _robot_art_holder[0] is not None:
        try:
            import math, numpy as np
            pos0, q0 = _robot_art_holder[0].get_world_pose()
            # 初始 yaw（Isaac quat = [qw,qx,qy,qz] scalar-first）
            qw0,qx0,qy0,qz0 = float(q0[0]),float(q0[1]),float(q0[2]),float(q0[3])
            yaw0 = math.atan2(2*(qw0*qz0+qx0*qy0), 1-2*(qy0*qy0+qz0*qz0))
            _odom_origin[0] = (float(pos0[0]), float(pos0[1]), yaw0)
            print(f"[isaac_scene] odom 原点: x={pos0[0]:.3f} y={pos0[1]:.3f} yaw={math.degrees(yaw0):.1f}°", flush=True)
        except Exception as e:
            print(f"[isaac_scene] ⚠️  初始位姿读取失败: {e}", flush=True)

    # 纯 Python cmd_vel 订阅 + odom/TF 发布
    _cmd_linear  = [0.0]
    _cmd_angular = [0.0]
    _ros_node    = [None]
    _odom_pub    = [None]
    _tf_broadcaster = [None]
    _pos_x = [0.0]; _pos_y = [0.0]; _yaw = [0.0]
    _last_t = [None]
    _last_stamp_ns = [0]   # 单调时间戳抑制 TF_OLD_DATA
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        import tf2_ros, math
        if not rclpy.ok():
            rclpy.init()
        _ros_node[0] = rclpy.create_node(
            "isaac_bridge",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)
            ]
        )
        def _cb(msg):
            _cmd_linear[0]  = msg.linear.x
            _cmd_angular[0] = msg.angular.z
        _ros_node[0].create_subscription(Twist, "/cmd_vel", _cb, 10)
        _odom_pub[0] = _ros_node[0].create_publisher(Odometry, "/odom", 10)
        _tf_broadcaster[0] = tf2_ros.TransformBroadcaster(_ros_node[0])
        print("[isaac_scene] ✅ /cmd_vel 订阅、/odom 发布、TF 就绪 (Python rclpy)", flush=True)
    except Exception as e:
        print(f"[isaac_scene] ⚠️  rclpy 初始化失败: {e}", flush=True)

    WHEEL_RADIUS = 0.0625   # 驱动轮半径 0.0625m (Φ125mm)
    WHEEL_BASE   = 0.3507   # 轮距 0.17535×2 m
    _PHYS_DT     = 1.0 / 60.0   # 物理步长（秒）：限制仿真速度 ≈ 1× 真实时间

    print("[isaac_scene] 主循环启动", flush=True)
    import math, time as _time
    import numpy as np
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time as RclpyTime

    while app.is_running():
        _loop_start = _time.monotonic()

        # ── 1. 消费 ROS 消息（cmd_vel 等） ──────────────────────────
        if _ros_node[0] is not None:
            import rclpy
            rclpy.spin_once(_ros_node[0], timeout_sec=0)

        # ── 2. 下发车轮速度（physics 步前写入指令） ────────────────
        if _art_ctrl[0] is not None and _wheel_idx[0] is not None:
            try:
                from isaacsim.core.utils.types import ArticulationAction
                vl = (_cmd_linear[0] - _cmd_angular[0] * WHEEL_BASE / 2.0) / WHEEL_RADIUS
                vr = (_cmd_linear[0] + _cmd_angular[0] * WHEEL_BASE / 2.0) / WHEEL_RADIUS
                li, ri = _wheel_idx[0]
                _art_ctrl[0].apply_action(ArticulationAction(
                    joint_velocities=np.array([vl, vr]),
                    joint_indices=np.array([li, ri]),
                ))
            except Exception:
                pass

        # render=True 触发 OmniGraph（/clock /lidar/points）
        # 若无 UI 可改为 render=False 节省 GPU 资源
        world.step(render=not _HEADLESS)

        # ── 4. 读取仿真时间（与 OmniGraph /clock 同一帧，无时序差）
        if _odom_pub[0] is None:
            # 限速
            _loop_end = _time.monotonic()
            _sleep = _PHYS_DT - (_loop_end - _loop_start)
            if _sleep > 0:
                _time.sleep(_sleep)
            continue

        try:
            # world.current_time 与 OmniGraph IsaacReadSimulationTime 同源
            sim_t    = world.current_time
            sim_sec  = int(sim_t)
            sim_nsec = int((sim_t - sim_sec) * 1e9)
            now_ns   = sim_sec * 1_000_000_000 + sim_nsec

            # sim_time 归零检测：仿真重启时 world.current_time 会从大应回到小
            # 必须重置 _last_stamp_ns，否则 now_ns 永远 <= 旧大値 → TF 永久卡死
            if now_ns < _last_stamp_ns[0] - 1_000_000_000:   # 后退超过 1科 → 判定为重启
                print(f"[isaac_scene] sim_time 归零检测，重置 TF 时间戳: {_last_stamp_ns[0]/1e9:.2f}→{now_ns/1e9:.2f}", flush=True)
                _last_stamp_ns[0] = 0

            # dt 计算（时间未推进时用上一帧 dt）
            if now_ns > _last_stamp_ns[0]:
                dt = (now_ns - _last_stamp_ns[0]) * 1e-9 if _last_stamp_ns[0] > 0 else _PHYS_DT
                _last_stamp_ns[0] = now_ns
            else:
                # 时间没推进：使用同一时间戳重新发布 TF，确保点云的时间戳 T 始终有对应 TF
                # tf2 对重复时间戳只会输出警告，不会导致 RViz 丢帧
                dt = _PHYS_DT
            stamp = RclpyTime(nanoseconds=now_ns).to_msg()

            # ── 5. 读取真实位姿，计算 odom ─────────────────────────
            use_physics = False
            if _robot_art_holder[0] is not None:
                try:
                    pos, quat = _robot_art_holder[0].get_world_pose()
                    # Isaac 四元数格式 scalar-first: [qw, qx, qy, qz]
                    qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

                    # 减去 odom 原点偏移
                    if _odom_origin[0] is not None:
                        ox, oy, oyaw = _odom_origin[0]
                        dx = float(pos[0]) - ox
                        dy = float(pos[1]) - oy
                        c, s = math.cos(-oyaw), math.sin(-oyaw)
                        px = c*dx - s*dy
                        py = s*dx + c*dy
                    else:
                        px, py = float(pos[0]), float(pos[1])

                    # 轮速反算底盘速度，积分 yaw
                    joint_vels = _robot_art_holder[0].get_joint_velocities()
                    li, ri = _wheel_idx[0]
                    vl_a = float(joint_vels[li]) * WHEEL_RADIUS
                    vr_a = float(joint_vels[ri]) * WHEEL_RADIUS
                    vx  = (vl_a + vr_a) / 2.0
                    wz  = (vr_a - vl_a) / WHEEL_BASE
                    _yaw[0]   += wz * dt
                    _pos_x[0]  = px
                    _pos_y[0]  = py
                    qz_out = math.sin(_yaw[0] / 2)
                    qw_out = math.cos(_yaw[0] / 2)
                    use_physics = True
                except Exception:
                    pass

            if not use_physics:
                vx = _cmd_linear[0]
                wz = _cmd_angular[0]
                _yaw[0]   += wz * dt
                _pos_x[0] += vx * math.cos(_yaw[0]) * dt
                _pos_y[0] += vx * math.sin(_yaw[0]) * dt
                qz_out = math.sin(_yaw[0] / 2)
                qw_out = math.cos(_yaw[0] / 2)

            # ── 6. 发布 /odom ───────────────────────────────────────
            odom_msg = Odometry()
            odom_msg.header.stamp    = stamp
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id  = "base_footprint"
            odom_msg.pose.pose.position.x    = _pos_x[0]
            odom_msg.pose.pose.position.y    = _pos_y[0]
            odom_msg.pose.pose.orientation.z = qz_out
            odom_msg.pose.pose.orientation.w = qw_out
            odom_msg.twist.twist.linear.x    = vx
            odom_msg.twist.twist.angular.z   = wz
            _odom_pub[0].publish(odom_msg)

            # ── 7. 发布 TF odom → base_footprint ───────────────────
            t = TransformStamped()
            t.header.stamp    = stamp
            t.header.frame_id = "odom"
            t.child_frame_id  = "base_footprint"
            t.transform.translation.x = _pos_x[0]
            t.transform.translation.y = _pos_y[0]
            t.transform.translation.z = 0.0
            t.transform.rotation.z    = qz_out
            t.transform.rotation.w    = qw_out
            _tf_broadcaster[0].sendTransform(t)

        except Exception:
            pass

        # ── 8. 限速 ≈ 1× 真实时间 ───────────────────────────────────
        _loop_end = _time.monotonic()
        _sleep = _PHYS_DT - (_loop_end - _loop_start)
        if _sleep > 0:
            _time.sleep(_sleep)

    if _ros_node[0] is not None:
        _ros_node[0].destroy_node()
    app.close()


if __name__ == "__main__":
    main()
