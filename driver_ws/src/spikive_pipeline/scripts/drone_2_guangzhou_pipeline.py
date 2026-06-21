#!/usr/bin/env python3
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import time
import copy
from bisect import bisect_left
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rosbag
import rosparam
import rospkg
import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


WIN_SIZE = 10
DEFAULT_MATCH_TOLERANCE_SEC = 1e-4
FLOAT32_DATATYPE = 7
DEFAULT_SLAM_EXTRINSIC_R = [
    0.9171,
    0.0,
    0.3987,
    0.0,
    1.0,
    0.0,
    -0.3987,
    0.0,
    0.9171,
]
DEFAULT_SLAM_EXTRINSIC_T = [0.065, 0.0, 0.071]
_NOT_SET = object()
PGO_STD_PARAM_DEFAULTS: Dict[str, object] = {
    "skip_near_num": 20,
    "candidate_num": 30,
    "sub_frame_num": 10,
    "center_dis_threshold": 80.0,
    "rough_dis_threshold": 0.02,
    "vertex_diff_threshold": 0.7,
    "icp_threshold": 0.35,
    "normal_threshold": 0.2,
    "dis_threshold": 0.5,
}
PGO_STD_INT_PARAMS = {"skip_near_num", "candidate_num", "sub_frame_num"}


def merge_dicts(base: Dict[str, object], override: Dict[str, object]) -> Dict[str, object]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_dicts(merged[key], value)  # type: ignore[arg-type]
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def param_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


@dataclass
class PoseRecord:
    stamp: float
    tx: float
    ty: float
    tz: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class StageResult:
    name: str
    returncode: int
    duration_sec: float
    log_path: str
    output: str
    status: str = "ok"


@dataclass
class LocalExtrinsic:
    apply: bool
    rotation: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
    translation: Tuple[float, float, float]


@dataclass
class StageBagStats:
    clouds_total: int = 0
    poses_total: int = 0
    frames_written: int = 0
    local_clouds_written: int = 0
    optimized_clouds_written: int = 0
    odom_written: int = 0
    map_written: int = 0
    max_pose_dt_sec: float = 0.0


def q_normalized(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def rotate_point(
    x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    # q * p * q^-1, expanded to avoid numpy/scipy dependencies.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def package_dir() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def project_dir() -> str:
    env_root = os.environ.get("POINTCLOUD_PGOBA_ROOT")
    if env_root:
        return os.path.abspath(os.path.expanduser(env_root))

    real_package_dir = os.path.realpath(package_dir())
    for candidate in (
        os.path.abspath(os.path.join(real_package_dir, "..", "..", "..")),
        os.path.abspath(os.path.join(package_dir(), "..", "..", "..")),
    ):
        if os.path.isdir(os.path.join(candidate, "driver_ws", "src")):
            return candidate
    return os.path.abspath(os.path.join(real_package_dir, "..", "..", ".."))


def default_pointcloud_process_config_path() -> str:
    return os.path.join(package_dir(), "config", "pointcloud_process_config.yaml")


def default_pipeline_config_path() -> str:
    return os.path.join(package_dir(), "config", "drone_2_guangzhou_pipeline.yaml")


def load_yaml_config(path: str) -> Dict[str, object]:
    if not path:
        return {}
    if not os.path.exists(path):
        rospy.logwarn("Configured YAML file does not exist: %s", path)
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML config root must be a mapping: {path}")
    return data


def default_pipeline_config() -> Dict[str, object]:
    return {
        "reference": {
            "pointcloud_process_config": default_pointcloud_process_config_path(),
        },
        "input": {
            "bag_path": "input/drone_2_gld_indexed.bag",
            "cloud_topic": "/cloud_registered_body",
            "odom_topic": "/drone_2_visual_slam/odom",
        },
        "output": {
            "root": os.path.join(project_dir(), "output"),
            "world_frame_id": "world",
            "stitch_pcd": True,
            "map_resolution": 0.0,
            "write_stage_local_clouds": False,
            "write_optimized_frame_clouds": True,
        },
        "pgo": {
            "input_cloud_topic": "/cloud_registered_body",
            "input_odom_topic": "/Odometry_pipeline_input",
            "save_name": "pgo",
            "use_loop": True,
            "use_gps": False,
            "use_key_frame": True,
            "key_frame_len_thre": 0.3,
            "key_frame_ang_thre": 0.1,
            "std": copy.deepcopy(PGO_STD_PARAM_DEFAULTS),
            "run_native": True,
        },
        "hba": {
            "run_native": True,
            "total_layer_num": 3,
            "thread_num": 4,
        },
        "runtime": {
            "gtsam_lib_dir": "/workspace/catkin_ws/third_party/install/lib",
            "pgo_gtsam_lib_dir": "/workspace/catkin_ws/third_party/gtsam-4.0.2/lib",
            "hba_gtsam_lib_dir": "/workspace/catkin_ws/third_party/gtsam-4.1/lib",
        },
        "evaluation": {
            "mme_thr_num": 4,
        },
        "extrinsic": {
            "apply": True,
            "pose_mode": "right_multiply",
            "R": DEFAULT_SLAM_EXTRINSIC_R,
            "T": DEFAULT_SLAM_EXTRINSIC_T,
        },
    }


def config_get(config: Dict[str, object], dotted_key: str, fallback):
    value = config
    for key in dotted_key.split("."):
        if not isinstance(value, dict) or key not in value:
            return fallback
        value = value[key]
    return value


def private_param(name: str, fallback):
    return rospy.get_param(f"~{name}", fallback)


def private_param_optional(name: str):
    param_name = f"~{name}"
    if not rospy.has_param(param_name):
        return _NOT_SET
    value = rospy.get_param(param_name)
    if value is None or value == "":
        return _NOT_SET
    return value


def resolved_param(
    name: str,
    pipeline_config: Dict[str, object],
    pipeline_key: str,
    fallback,
    pointcloud_config: Optional[Dict[str, object]] = None,
    pointcloud_key: Optional[str] = None,
):
    value = private_param_optional(name)
    if value is not _NOT_SET:
        return value
    value = config_get(pipeline_config, pipeline_key, _NOT_SET)
    if value is not _NOT_SET:
        return value
    if pointcloud_config is not None and pointcloud_key:
        value = config_get(pointcloud_config, pointcloud_key, _NOT_SET)
        if value is not _NOT_SET:
            return value
    return fallback


def resolve_pgo_std_params(pipeline_config: Dict[str, object]) -> Dict[str, object]:
    params: Dict[str, object] = {}
    for key, fallback in PGO_STD_PARAM_DEFAULTS.items():
        value = resolved_param(
            f"pgo_std_{key}",
            pipeline_config,
            f"pgo.std.{key}",
            fallback,
        )
        if key in PGO_STD_INT_PARAMS:
            value = int(value)
            if value <= 0:
                raise RuntimeError(f"pgo.std.{key} must be positive, got {value}")
        else:
            value = float(value)
            if not math.isfinite(value):
                raise RuntimeError(f"pgo.std.{key} must be finite, got {value}")
        params[key] = value
    return params


def resolve_relative_path(path_value, base_dir: str) -> str:
    path = os.path.expanduser(str(path_value))
    if not path:
        return path
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(base_dir, path))


def parse_float_list(value, expected_size: int, name: str) -> List[float]:
    if isinstance(value, str):
        try:
            value = yaml.safe_load(value)
        except yaml.YAMLError as exc:
            raise RuntimeError(f"Invalid list parameter {name}: {exc}") from exc
    array = np.asarray(value, dtype=np.float64)
    if array.shape == (3, 3):
        array = array.reshape(-1)
    if array.shape != (expected_size,):
        raise RuntimeError(f"{name} expects {expected_size} numbers, got shape {array.shape}")
    if not np.all(np.isfinite(array)):
        raise RuntimeError(f"{name} contains non-finite values")
    return [float(v) for v in array.tolist()]


def make_local_extrinsic(apply: bool, rotation_values, translation_values) -> LocalExtrinsic:
    rotation_flat = parse_float_list(rotation_values, 9, "slam_stitch_extrinsic_r")
    translation = parse_float_list(translation_values, 3, "slam_stitch_extrinsic_t")
    if not apply:
        rotation_flat = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        translation = [0.0, 0.0, 0.0]
    return LocalExtrinsic(
        apply=bool(apply),
        rotation=(
            (rotation_flat[0], rotation_flat[1], rotation_flat[2]),
            (rotation_flat[3], rotation_flat[4], rotation_flat[5]),
            (rotation_flat[6], rotation_flat[7], rotation_flat[8]),
        ),
        translation=(translation[0], translation[1], translation[2]),
    )


def is_identity_extrinsic(extrinsic: LocalExtrinsic) -> bool:
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    zero = (0.0, 0.0, 0.0)
    return (
        not extrinsic.apply
        or (
            np.allclose(np.asarray(extrinsic.rotation), np.asarray(identity), atol=1e-12)
            and np.allclose(np.asarray(extrinsic.translation), np.asarray(zero), atol=1e-12)
        )
    )


def apply_local_extrinsic(
    x: float,
    y: float,
    z: float,
    extrinsic: LocalExtrinsic,
) -> Tuple[float, float, float]:
    r = extrinsic.rotation
    t = extrinsic.translation
    return (
        r[0][0] * x + r[0][1] * y + r[0][2] * z + t[0],
        r[1][0] * x + r[1][1] * y + r[1][2] * z + t[1],
        r[2][0] * x + r[2][1] * y + r[2][2] * z + t[2],
    )


def q_multiply(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalized(
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    qx, qy, qz, qw = q_normalized(*q)
    return -qx, -qy, -qz, qw


def rotation_matrix_to_quaternion(
    rotation: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
) -> Tuple[float, float, float, float]:
    m = rotation
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(max(0.0, 1.0 + m[0][0] - m[1][1] - m[2][2])) * 2.0
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(max(0.0, 1.0 + m[1][1] - m[0][0] - m[2][2])) * 2.0
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + m[2][2] - m[0][0] - m[1][1])) * 2.0
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    return q_normalized(qx, qy, qz, qw)


def transform_pose_with_extrinsic(
    pose: PoseRecord,
    extrinsic: LocalExtrinsic,
    mode: str,
) -> PoseRecord:
    if is_identity_extrinsic(extrinsic):
        return pose

    mode = (mode or "right_multiply").strip().lower()
    pose_q = q_normalized(pose.qx, pose.qy, pose.qz, pose.qw)
    ext_q = rotation_matrix_to_quaternion(extrinsic.rotation)
    ext_t = extrinsic.translation
    pose_t = (pose.tx, pose.ty, pose.tz)

    if mode in ("right_multiply", "pose_right", "compose", "pose_times_extrinsic"):
        rotated_t = rotate_point(ext_t[0], ext_t[1], ext_t[2], *pose_q)
        out_t = (
            pose_t[0] + rotated_t[0],
            pose_t[1] + rotated_t[1],
            pose_t[2] + rotated_t[2],
        )
        out_q = q_multiply(pose_q, ext_q)
    elif mode in ("left_multiply", "pose_left", "extrinsic_times_pose"):
        rotated_t = rotate_point(pose_t[0], pose_t[1], pose_t[2], *ext_q)
        out_t = (
            rotated_t[0] + ext_t[0],
            rotated_t[1] + ext_t[1],
            rotated_t[2] + ext_t[2],
        )
        out_q = q_multiply(ext_q, pose_q)
    elif mode in ("conjugate", "extrinsic_conjugate"):
        ext_inv_q = q_inverse(ext_q)
        minus_ext_t = (-ext_t[0], -ext_t[1], -ext_t[2])
        inv_t = rotate_point(minus_ext_t[0], minus_ext_t[1], minus_ext_t[2], *ext_inv_q)
        intermediate_q = q_multiply(ext_q, pose_q)
        intermediate_t = rotate_point(pose_t[0], pose_t[1], pose_t[2], *ext_q)
        intermediate_t = (
            intermediate_t[0] + ext_t[0],
            intermediate_t[1] + ext_t[1],
            intermediate_t[2] + ext_t[2],
        )
        rotated_inv_t = rotate_point(inv_t[0], inv_t[1], inv_t[2], *intermediate_q)
        out_t = (
            intermediate_t[0] + rotated_inv_t[0],
            intermediate_t[1] + rotated_inv_t[1],
            intermediate_t[2] + rotated_inv_t[2],
        )
        out_q = q_multiply(intermediate_q, ext_inv_q)
    else:
        raise RuntimeError(
            "Unsupported slam_stitch_extrinsic_pose_mode "
            f"{mode!r}; expected right_multiply, left_multiply, or conjugate"
        )

    return PoseRecord(
        pose.stamp,
        out_t[0],
        out_t[1],
        out_t[2],
        out_q[0],
        out_q[1],
        out_q[2],
        out_q[3],
    )


def ensure_clean_output_root(output_root: str) -> Dict[str, str]:
    output_root = os.path.abspath(output_root)
    if output_root in ("/", "/tmp", "/home", os.path.expanduser("~")):
        raise RuntimeError(f"Refusing unsafe output_root: {output_root}")

    os.makedirs(output_root, exist_ok=True)

    generated_dirs = ["input", "pgo_input", "pgo_output", "hba", "logs"]
    generated_files = ["summary.yaml", "evaluation.txt", "optimization_evaluation.yaml"]
    for dirname in generated_dirs:
        path = os.path.join(output_root, dirname)
        if os.path.exists(path):
            shutil.rmtree(path)
    for filename in generated_files:
        path = os.path.join(output_root, filename)
        if os.path.exists(path):
            os.remove(path)

    paths = {
        "root": output_root,
        "input": os.path.join(output_root, "input"),
        "pgo_input": os.path.join(output_root, "pgo_input"),
        "pgo_output": os.path.join(output_root, "pgo_output"),
        "hba": os.path.join(output_root, "hba"),
        "hba_pcd": os.path.join(output_root, "hba", "pcd"),
        "logs": os.path.join(output_root, "logs"),
    }
    for path in paths.values():
        os.makedirs(path, exist_ok=True)
    return paths


def bag_stem(bag_path: str) -> str:
    stem = os.path.splitext(os.path.basename(bag_path))[0]
    return re.sub(r"[^A-Za-z0-9_]+", "_", stem)


def export_pose_file(
    bag_path: str,
    odom_topic: str,
    cloud_topic: str,
    pose_path: str,
    extrinsic: Optional[LocalExtrinsic] = None,
    extrinsic_pose_mode: str = "right_multiply",
) -> Tuple[int, int]:
    odom_count = 0
    cloud_count = 0
    with rosbag.Bag(bag_path, "r") as bag, open(pose_path, "w") as pose_file:
        for topic, msg, _ in bag.read_messages(topics=[odom_topic, cloud_topic]):
            if topic == cloud_topic:
                cloud_count += 1
                continue
            if topic != odom_topic:
                continue

            qx, qy, qz, qw = q_normalized(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            pose = PoseRecord(
                msg.header.stamp.to_sec(),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                qx,
                qy,
                qz,
                qw,
            )
            if extrinsic is not None:
                pose = transform_pose_with_extrinsic(pose, extrinsic, extrinsic_pose_mode)
            pose_file.write(
                f"{pose.stamp:.20f} "
                f"{pose.tx:.20f} "
                f"{pose.ty:.20f} "
                f"{pose.tz:.20f} "
                f"{pose.qx:.20f} {pose.qy:.20f} {pose.qz:.20f} {pose.qw:.20f}\n"
            )
            odom_count += 1

    if odom_count == 0:
        raise RuntimeError(f"No odometry messages found on {odom_topic}")
    if cloud_count == 0:
        raise RuntimeError(f"No cloud messages found on {cloud_topic}")
    return odom_count, cloud_count


def load_rosparam_file(path: str, namespace: str = "/") -> None:
    for params, ns in rosparam.load_file(path, default_namespace=namespace):
        rosparam.upload_params(ns, params)


def configure_pgo_params(
    pgo_config_path: str,
    pgo_input: str,
    pgo_output: str,
    input_name: str,
    pgo_save_name: str,
    cloud_topic: str,
    use_loop: bool,
    use_gps: bool,
    use_key_frame: bool,
    key_frame_len_thre: float,
    key_frame_ang_thre: float,
    map_resolution: float,
    pgo_std_params: Dict[str, object],
) -> None:
    load_rosparam_file(pgo_config_path, "/")
    rospy.set_param("/run_pgo", True)
    rospy.set_param("/use_loop", bool(use_loop))
    rospy.set_param("/use_gps", bool(use_gps))
    rospy.set_param("/loop/use_key_frame", bool(use_key_frame))
    rospy.set_param("/loop/key_frame_len_thre", float(key_frame_len_thre))
    rospy.set_param("/loop/key_frame_ang_thre", float(key_frame_ang_thre))
    rospy.set_param("/use_file", False)
    rospy.set_param("/cloud_topic", cloud_topic)
    rospy.set_param("/odom_topic", "/unused_odom_topic_for_file_mode")
    rospy.set_param("/gps_topic", "/unused_gps_topic")
    rospy.set_param("/input_path", pgo_input)
    rospy.set_param("/input_bag_names", input_name)
    rospy.set_param("/input_pose_names", input_name)
    rospy.set_param("/input_loop_names", "")
    rospy.set_param("/save_path", pgo_output)
    rospy.set_param("/save_name", pgo_save_name)
    rospy.set_param("/map_resolution", float(map_resolution))
    for key, value in pgo_std_params.items():
        rospy.set_param(f"/{key}", value)
    rospy.loginfo(
        "PGO STD params: %s",
        ", ".join(f"{key}={pgo_std_params[key]}" for key in sorted(pgo_std_params)),
    )


def run_logged(
    name: str,
    cmd: List[str],
    log_dir: str,
    cwd: Optional[str] = None,
    check: bool = True,
    env: Optional[Dict[str, str]] = None,
) -> StageResult:
    log_path = os.path.join(log_dir, f"{name}.log")
    start = time.time()
    rospy.loginfo("Running %s: %s", name, " ".join(cmd))

    with open(log_path, "w") as log_file:
        proc = subprocess.Popen(
            cmd,
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
        )
        captured: List[str] = []
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                log_file.write(line)
                if len(captured) < 4000:
                    captured.append(line)
            proc.wait()
        except BaseException:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            raise

    duration = time.time() - start
    output = "".join(captured)
    result = StageResult(name, proc.returncode, duration, log_path, output)
    if proc.returncode != 0 and check:
        raise RuntimeError(
            f"Stage {name} failed with code {proc.returncode}. See log: {log_path}"
        )
    if proc.returncode != 0:
        result.status = "failed"
        rospy.logwarn("Stage %s failed with code %d; log: %s", name, proc.returncode, log_path)
        return result
    rospy.loginfo("Finished %s in %.2fs", name, duration)
    return result


def read_pgo_pose(path: str) -> List[PoseRecord]:
    poses: List[PoseRecord] = []
    with open(path, "r") as f:
        for line_no, line in enumerate(f, 1):
            parts = line.strip().split()
            if not parts:
                continue
            if len(parts) != 8:
                raise RuntimeError(f"Invalid PGO pose line {line_no}: {line.rstrip()}")
            stamp, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
            qx, qy, qz, qw = q_normalized(qx, qy, qz, qw)
            poses.append(PoseRecord(stamp, tx, ty, tz, qx, qy, qz, qw))
    if not poses:
        raise RuntimeError(f"No poses found in {path}")
    return poses


def write_hba_pose(path: str, poses: List[PoseRecord]) -> None:
    # HBA's reader is eof-sensitive; do not emit a trailing newline.
    with open(path, "w") as f:
        for i, pose in enumerate(poses):
            f.write(
                f"{pose.tx:.12f} {pose.ty:.12f} {pose.tz:.12f} "
                f"{pose.qw:.12f} {pose.qx:.12f} {pose.qy:.12f} {pose.qz:.12f}"
            )
            if i < len(poses) - 1:
                f.write("\n")


def write_xyz_pcd(path: str, points: Iterable[Tuple[float, float, float]]) -> int:
    point_list = list(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(point_list)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(point_list)}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for x, y, z in point_list:
            f.write(struct.pack("<fff", float(x), float(y), float(z)))
    return len(point_list)


def write_xyzi_pcd(path: str, points: Iterable[Tuple[float, float, float, float]]) -> int:
    point_list = list(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {len(point_list)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(point_list)}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        for x, y, z, intensity in point_list:
            f.write(struct.pack("<ffff", float(x), float(y), float(z), float(intensity)))
    return len(point_list)


def xyzi_points_to_cloud_msg(
    points: Iterable[Tuple[float, float, float, float]],
    stamp: rospy.Time,
    frame_id: str = "camera_init",
) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    fields = [
        PointField("x", 0, FLOAT32_DATATYPE, 1),
        PointField("y", 4, FLOAT32_DATATYPE, 1),
        PointField("z", 8, FLOAT32_DATATYPE, 1),
        PointField("intensity", 12, FLOAT32_DATATYPE, 1),
    ]
    return pc2.create_cloud(header, fields, list(points))


def pose_to_odom_msg(
    pose: PoseRecord,
    topic_frame_id: str = "camera_init",
    child_frame_id: str = "body",
) -> Odometry:
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(pose.stamp)
    msg.header.frame_id = topic_frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = pose.tx
    msg.pose.pose.position.y = pose.ty
    msg.pose.pose.position.z = pose.tz
    msg.pose.pose.orientation.x = pose.qx
    msg.pose.pose.orientation.y = pose.qy
    msg.pose.pose.orientation.z = pose.qz
    msg.pose.pose.orientation.w = pose.qw
    return msg


def pointcloud2_xyz_points(msg) -> Iterable[Tuple[float, float, float]]:
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z


def pointcloud2_xyzi_points(msg) -> Iterable[Tuple[float, float, float, float]]:
    field_names = [field.name for field in msg.fields]
    use_intensity = "intensity" in field_names
    names = ("x", "y", "z", "intensity") if use_intensity else ("x", "y", "z")
    for point in pc2.read_points(msg, field_names=names, skip_nans=True):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        intensity = float(point[3]) if use_intensity else 0.0
        if not math.isfinite(intensity):
            intensity = 0.0
        yield x, y, z, intensity


def nearest_pose(
    poses: List[PoseRecord],
    stamps: List[float],
    stamp: float,
    tolerance_sec: float,
) -> Tuple[Optional[PoseRecord], float]:
    index = bisect_left(stamps, stamp)
    candidates: List[PoseRecord] = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    if not candidates:
        return None, 0.0
    pose = min(candidates, key=lambda item: abs(item.stamp - stamp))
    dt = abs(pose.stamp - stamp)
    if dt > tolerance_sec:
        return None, dt
    return pose, dt


def optimized_points_from_msg(
    msg,
    pose: PoseRecord,
) -> Iterable[Tuple[float, float, float, float]]:
    for x, y, z, intensity in pointcloud2_xyzi_points(msg):
        rx, ry, rz = rotate_point(x, y, z, pose.qx, pose.qy, pose.qz, pose.qw)
        yield rx + pose.tx, ry + pose.ty, rz + pose.tz, intensity


def write_pgo_input_bag(
    source_bag_path: str,
    output_bag_path: str,
    source_cloud_topic: str,
    source_odom_topic: str,
    output_cloud_topic: str,
    output_odom_topic: str,
) -> Tuple[int, int]:
    if os.path.exists(output_bag_path):
        os.remove(output_bag_path)

    cloud_count = 0
    odom_count = 0
    with rosbag.Bag(source_bag_path, "r") as source, rosbag.Bag(output_bag_path, "w") as out:
        for topic, msg, t in source.read_messages(topics=[source_cloud_topic, source_odom_topic]):
            if topic == source_cloud_topic:
                out.write(
                    output_cloud_topic,
                    msg,
                    t=msg.header.stamp,
                )
                cloud_count += 1
            elif topic == source_odom_topic:
                out.write(output_odom_topic, msg, t=msg.header.stamp)
                odom_count += 1

    if cloud_count == 0:
        raise RuntimeError(f"No cloud messages found on {source_cloud_topic}")
    if odom_count == 0:
        raise RuntimeError(f"No odometry messages found on {source_odom_topic}")
    return odom_count, cloud_count


def write_optimized_stage_bag(
    source_bag_path: str,
    output_bag_path: str,
    source_cloud_topic: str,
    poses: List[PoseRecord],
    output_cloud_topic: str,
    output_odom_topic: str,
    output_map_topic: str,
    frame_id: str,
    write_local_clouds: bool,
    write_optimized_clouds: bool,
    stitch_pcd: bool,
    map_path: str,
    map_resolution: float,
    tolerance_sec: float = DEFAULT_MATCH_TOLERANCE_SEC,
) -> StageBagStats:
    if os.path.exists(output_bag_path):
        os.remove(output_bag_path)

    stamps = [pose.stamp for pose in poses]
    stats = StageBagStats(poses_total=len(poses))
    voxels: Dict[Tuple[int, int, int], Tuple[float, float, float, float, int]] = {}
    raw_map_points: List[Tuple[float, float, float, float]] = []
    def add_map_point(wx: float, wy: float, wz: float, intensity: float) -> None:
        if map_resolution > 0.0:
            key = (
                math.floor(wx / map_resolution),
                math.floor(wy / map_resolution),
                math.floor(wz / map_resolution),
            )
            sx, sy, sz, si, count = voxels.get(key, (0.0, 0.0, 0.0, 0.0, 0))
            voxels[key] = (sx + wx, sy + wy, sz + wz, si + intensity, count + 1)
        else:
            raw_map_points.append((wx, wy, wz, intensity))

    with rosbag.Bag(source_bag_path, "r") as source, rosbag.Bag(output_bag_path, "w") as out:
        for topic, msg, _ in source.read_messages(topics=[source_cloud_topic]):
            if topic != source_cloud_topic:
                continue
            stats.clouds_total += 1
            stamp = msg.header.stamp.to_sec()
            pose, dt = nearest_pose(poses, stamps, stamp, tolerance_sec)
            stats.max_pose_dt_sec = max(stats.max_pose_dt_sec, dt)
            if pose is None:
                continue

            stats.frames_written += 1
            odom_msg = pose_to_odom_msg(pose, topic_frame_id=frame_id, child_frame_id="body")
            out.write(output_odom_topic, odom_msg, t=msg.header.stamp)
            stats.odom_written += 1

            if write_local_clouds:
                out.write(source_cloud_topic, msg, t=msg.header.stamp)
                stats.local_clouds_written += 1

            if write_optimized_clouds or stitch_pcd:
                optimized_points = list(optimized_points_from_msg(msg, pose))
            else:
                optimized_points = []

            if write_optimized_clouds:
                cloud_msg = xyzi_points_to_cloud_msg(
                    optimized_points,
                    msg.header.stamp,
                    frame_id=frame_id,
                )
                out.write(output_cloud_topic, cloud_msg, t=msg.header.stamp)
                stats.optimized_clouds_written += 1

            if stitch_pcd:
                for wx, wy, wz, intensity in optimized_points:
                    add_map_point(wx, wy, wz, intensity)

        if stitch_pcd:
            if map_resolution > 0.0:
                map_points = [
                    (sx / count, sy / count, sz / count, si / count)
                    for sx, sy, sz, si, count in voxels.values()
                ]
            else:
                map_points = raw_map_points
            stats.map_written = len(map_points)
            if stats.map_written == 0:
                raise RuntimeError(f"No points available for stitched map from {source_bag_path}")
            map_msg = xyzi_points_to_cloud_msg(
                map_points,
                rospy.Time.from_sec(poses[-1].stamp),
                frame_id=frame_id,
            )
            write_xyzi_pcd(map_path, map_points)
            out.write(output_map_topic, map_msg, t=map_msg.header.stamp)

    if stats.frames_written == 0:
        raise RuntimeError(
            f"No clouds in {source_bag_path} matched {len(poses)} poses on {source_cloud_topic}"
        )
    if stats.frames_written != len(poses):
        raise RuntimeError(
            f"Matched {stats.frames_written}/{len(poses)} poses to clouds in {source_bag_path}"
        )
    return stats


def export_hba_dataset(
    bag_path: str,
    cloud_topic: str,
    poses: List[PoseRecord],
    hba_dir: str,
    pcd_dir: str,
    tolerance_sec: float = DEFAULT_MATCH_TOLERANCE_SEC,
) -> int:
    matched_poses: List[PoseRecord] = []
    pose_index = 0
    pcd_count = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[cloud_topic]):
            if topic != cloud_topic or pose_index >= len(poses):
                continue

            stamp = msg.header.stamp.to_sec()
            target = poses[pose_index].stamp
            while pose_index < len(poses) and target < stamp - tolerance_sec:
                raise RuntimeError(
                    f"Missing cloud for PGO pose stamp {target:.9f}; "
                    f"nearest later cloud is {stamp:.9f}"
                )
            if abs(stamp - target) > tolerance_sec:
                continue

            pcd_path = os.path.join(pcd_dir, f"{pcd_count}.pcd")
            point_count = write_xyz_pcd(pcd_path, pointcloud2_xyz_points(msg))
            if point_count == 0:
                raise RuntimeError(f"Cloud at {stamp:.9f} produced an empty PCD")
            matched_poses.append(poses[pose_index])
            pcd_count += 1
            pose_index += 1

    if pcd_count != len(poses):
        raise RuntimeError(f"Matched {pcd_count}/{len(poses)} PGO poses to clouds")
    if pcd_count < WIN_SIZE:
        raise RuntimeError(f"HBA requires at least {WIN_SIZE} poses, got {pcd_count}")

    pose_json = os.path.join(hba_dir, "pose.json")
    write_hba_pose(pose_json, matched_poses)
    shutil.copyfile(pose_json, os.path.join(hba_dir, "pose_pgo.json"))
    return pcd_count


def read_hba_pose(path: str) -> List[Tuple[float, float, float, float, float, float, float]]:
    poses: List[Tuple[float, float, float, float, float, float, float]] = []
    with open(path, "r") as f:
        for line_no, line in enumerate(f, 1):
            parts = line.strip().split()
            if not parts:
                continue
            if len(parts) != 7:
                raise RuntimeError(f"Invalid HBA pose line {line_no}: {line.rstrip()}")
            tx, ty, tz, qw, qx, qy, qz = map(float, parts)
            qx, qy, qz, qw = q_normalized(qx, qy, qz, qw)
            poses.append((tx, ty, tz, qx, qy, qz, qw))
    if not poses:
        raise RuntimeError(f"No HBA poses found in {path}")
    return poses


def hba_poses_with_stamps(path: str, stamps: List[float]) -> List[PoseRecord]:
    hba_poses = read_hba_pose(path)
    if len(hba_poses) != len(stamps):
        raise RuntimeError(
            f"HBA pose count {len(hba_poses)} does not match stamp count {len(stamps)}"
        )
    records: List[PoseRecord] = []
    for stamp, (tx, ty, tz, qx, qy, qz, qw) in zip(stamps, hba_poses):
        records.append(PoseRecord(stamp, tx, ty, tz, qx, qy, qz, qw))
    return records


def compose_pose_records(left: PoseRecord, right: PoseRecord, stamp: float) -> PoseRecord:
    left_q = q_normalized(left.qx, left.qy, left.qz, left.qw)
    right_q = q_normalized(right.qx, right.qy, right.qz, right.qw)
    rotated_right_t = rotate_point(right.tx, right.ty, right.tz, *left_q)
    out_q = q_multiply(left_q, right_q)
    return PoseRecord(
        stamp,
        left.tx + rotated_right_t[0],
        left.ty + rotated_right_t[1],
        left.tz + rotated_right_t[2],
        out_q[0],
        out_q[1],
        out_q[2],
        out_q[3],
    )


def inverse_pose_record(pose: PoseRecord, stamp: float) -> PoseRecord:
    inv_q = q_inverse((pose.qx, pose.qy, pose.qz, pose.qw))
    inv_t = rotate_point(-pose.tx, -pose.ty, -pose.tz, *inv_q)
    return PoseRecord(stamp, inv_t[0], inv_t[1], inv_t[2], inv_q[0], inv_q[1], inv_q[2], inv_q[3])


def expand_keyframe_poses_to_full_frames(
    keyframe_poses: List[PoseRecord],
    full_slam_poses: List[PoseRecord],
    tolerance_sec: float = DEFAULT_MATCH_TOLERANCE_SEC,
) -> List[PoseRecord]:
    if len(keyframe_poses) == len(full_slam_poses):
        return keyframe_poses
    if len(keyframe_poses) > len(full_slam_poses):
        raise RuntimeError(
            f"PGO pose count {len(keyframe_poses)} is larger than SLAM pose count "
            f"{len(full_slam_poses)}"
        )
    if not keyframe_poses:
        raise RuntimeError("Cannot expand empty PGO keyframe pose list")

    full_stamps = [pose.stamp for pose in full_slam_poses]
    keyframe_indices: List[int] = []
    last_index = -1
    for key_pose in keyframe_poses:
        index = bisect_left(full_stamps, key_pose.stamp)
        candidates = []
        if index < len(full_slam_poses):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        if not candidates:
            raise RuntimeError(f"PGO keyframe stamp has no matching SLAM pose: {key_pose.stamp}")
        best_index = min(candidates, key=lambda item: abs(full_stamps[item] - key_pose.stamp))
        dt = abs(full_stamps[best_index] - key_pose.stamp)
        if dt > tolerance_sec:
            raise RuntimeError(
                f"PGO keyframe stamp {key_pose.stamp:.9f} does not match a SLAM pose "
                f"within {tolerance_sec}s; nearest dt={dt:.9f}s"
            )
        if best_index <= last_index:
            raise RuntimeError("PGO keyframe poses are not strictly ordered in the SLAM timeline")
        keyframe_indices.append(best_index)
        last_index = best_index

    expanded: List[PoseRecord] = []
    anchor_key_index = 0
    for full_index, slam_pose in enumerate(full_slam_poses):
        while (
            anchor_key_index + 1 < len(keyframe_indices)
            and full_index >= keyframe_indices[anchor_key_index + 1]
        ):
            anchor_key_index += 1
        anchor_slam = full_slam_poses[keyframe_indices[anchor_key_index]]
        anchor_pgo = keyframe_poses[anchor_key_index]
        relative_slam = compose_pose_records(
            inverse_pose_record(anchor_slam, slam_pose.stamp),
            slam_pose,
            slam_pose.stamp,
        )
        expanded.append(compose_pose_records(anchor_pgo, relative_slam, slam_pose.stamp))
    return expanded


def anchor_hba_poses_to_world(
    hba_relative_poses: List[PoseRecord],
    pgo_world_poses: List[PoseRecord],
) -> List[PoseRecord]:
    if len(hba_relative_poses) != len(pgo_world_poses):
        raise RuntimeError(
            f"HBA pose count {len(hba_relative_poses)} does not match PGO pose count "
            f"{len(pgo_world_poses)}"
        )
    if not hba_relative_poses:
        return []
    anchor = PoseRecord(
        pgo_world_poses[0].stamp,
        pgo_world_poses[0].tx,
        pgo_world_poses[0].ty,
        pgo_world_poses[0].tz,
        pgo_world_poses[0].qx,
        pgo_world_poses[0].qy,
        pgo_world_poses[0].qz,
        pgo_world_poses[0].qw,
    )
    return [
        compose_pose_records(anchor, rel_pose, pgo_pose.stamp)
        for rel_pose, pgo_pose in zip(hba_relative_poses, pgo_world_poses)
    ]


def write_pgo_pose_file(path: str, poses: List[PoseRecord]) -> None:
    with open(path, "w") as f:
        for pose in poses:
            f.write(
                f"{pose.stamp:.20f} {pose.tx:.20f} {pose.ty:.20f} {pose.tz:.20f} "
                f"{pose.qx:.20f} {pose.qy:.20f} {pose.qz:.20f} {pose.qw:.20f}\n"
            )


def pose_translation_distance(left: PoseRecord, right: PoseRecord) -> float:
    return math.sqrt(
        (left.tx - right.tx) ** 2
        + (left.ty - right.ty) ** 2
        + (left.tz - right.tz) ** 2
    )


def pose_rotation_distance_rad(left: PoseRecord, right: PoseRecord) -> float:
    lq = q_normalized(left.qx, left.qy, left.qz, left.qw)
    rq = q_normalized(right.qx, right.qy, right.qz, right.qw)
    dot = abs(lq[0] * rq[0] + lq[1] * rq[1] + lq[2] * rq[2] + lq[3] * rq[3])
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def numeric_stats(values: List[float], prefix: str) -> Dict[str, float]:
    if not values:
        return {
            f"{prefix}_min": 0.0,
            f"{prefix}_mean": 0.0,
            f"{prefix}_p95": 0.0,
            f"{prefix}_max": 0.0,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        f"{prefix}_min": float(np.min(arr)),
        f"{prefix}_mean": float(np.mean(arr)),
        f"{prefix}_p95": float(np.percentile(arr, 95.0)),
        f"{prefix}_max": float(np.max(arr)),
    }


def pose_delta_stats(reference: List[PoseRecord], optimized: List[PoseRecord]) -> Dict[str, float]:
    if len(reference) != len(optimized):
        raise RuntimeError(
            f"Cannot compare pose lists with different sizes: {len(reference)} vs {len(optimized)}"
        )
    if not reference:
        raise RuntimeError("Cannot compare empty pose lists")

    translation_deltas: List[float] = []
    rotation_deltas: List[float] = []
    for before, after in zip(reference, optimized):
        translation_deltas.append(pose_translation_distance(before, after))
        rotation_deltas.append(pose_rotation_distance_rad(before, after))

    stats: Dict[str, float] = {}
    stats.update(numeric_stats(translation_deltas, "translation_delta"))
    stats.update(numeric_stats(rotation_deltas, "rotation_delta_rad"))
    stats.update({
        "translation_delta_gt_1cm": sum(v > 0.01 for v in translation_deltas),
        "rotation_delta_gt_1deg": sum(v > math.radians(1.0) for v in rotation_deltas),
    })
    return stats


def trajectory_stats(poses: List[PoseRecord]) -> Dict[str, float]:
    if not poses:
        raise RuntimeError("Cannot evaluate an empty trajectory")
    translation_steps = [
        pose_translation_distance(prev_pose, curr_pose)
        for prev_pose, curr_pose in zip(poses[:-1], poses[1:])
    ]
    rotation_steps = [
        pose_rotation_distance_rad(prev_pose, curr_pose)
        for prev_pose, curr_pose in zip(poses[:-1], poses[1:])
    ]
    stats: Dict[str, float] = {
        "duration_sec": poses[-1].stamp - poses[0].stamp,
        "path_length_m": float(sum(translation_steps)),
        "start_end_distance_m": pose_translation_distance(poses[0], poses[-1]),
    }
    stats.update(numeric_stats(translation_steps, "step_translation_m"))
    stats.update(numeric_stats(rotation_steps, "step_rotation_rad"))
    return stats


def relative_motion_delta_stats(
    reference: List[PoseRecord],
    optimized: List[PoseRecord],
) -> Dict[str, float]:
    if len(reference) != len(optimized):
        raise RuntimeError(
            f"Cannot compare relative motions with different sizes: "
            f"{len(reference)} vs {len(optimized)}"
        )
    if len(reference) < 2:
        raise RuntimeError("Cannot evaluate relative motion with fewer than 2 poses")

    translation_deltas: List[float] = []
    rotation_deltas: List[float] = []
    reference_steps: List[float] = []
    optimized_steps: List[float] = []
    for ref_prev, ref_curr, opt_prev, opt_curr in zip(
        reference[:-1], reference[1:], optimized[:-1], optimized[1:]
    ):
        stamp = ref_curr.stamp
        ref_rel = compose_pose_records(inverse_pose_record(ref_prev, stamp), ref_curr, stamp)
        opt_rel = compose_pose_records(inverse_pose_record(opt_prev, stamp), opt_curr, stamp)
        translation_deltas.append(pose_translation_distance(ref_rel, opt_rel))
        rotation_deltas.append(pose_rotation_distance_rad(ref_rel, opt_rel))
        reference_steps.append(
            math.sqrt(ref_rel.tx * ref_rel.tx + ref_rel.ty * ref_rel.ty + ref_rel.tz * ref_rel.tz)
        )
        optimized_steps.append(
            math.sqrt(opt_rel.tx * opt_rel.tx + opt_rel.ty * opt_rel.ty + opt_rel.tz * opt_rel.tz)
        )

    stats: Dict[str, float] = {}
    stats.update(numeric_stats(translation_deltas, "relative_translation_delta_m"))
    stats.update(numeric_stats(rotation_deltas, "relative_rotation_delta_rad"))
    reference_path = float(sum(reference_steps))
    optimized_path = float(sum(optimized_steps))
    stats.update({
        "relative_translation_delta_gt_1cm": sum(v > 0.01 for v in translation_deltas),
        "relative_rotation_delta_gt_1deg": sum(v > math.radians(1.0) for v in rotation_deltas),
        "reference_path_length_m": reference_path,
        "optimized_path_length_m": optimized_path,
        "path_length_delta_m": optimized_path - reference_path,
        "path_length_delta_percent": (
            100.0 * (optimized_path - reference_path) / reference_path
            if abs(reference_path) > 1e-12
            else 0.0
        ),
    })
    return stats


def read_xyz_pcd(path: str) -> Iterable[Tuple[float, float, float]]:
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError(f"PCD header ended unexpectedly: {path}")
            decoded = line.decode("ascii").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA "):
                data_type = decoded.split()[1]
                break

        fields = next((line.split()[1:] for line in header_lines if line.startswith("FIELDS ")), [])
        sizes = [int(v) for v in next((line.split()[1:] for line in header_lines if line.startswith("SIZE ")), [])]
        types = next((line.split()[1:] for line in header_lines if line.startswith("TYPE ")), [])
        counts = [
            int(v)
            for v in next(
                (line.split()[1:] for line in header_lines if line.startswith("COUNT ")),
                ["1"] * len(fields),
            )
        ]
        points_line = next((line for line in header_lines if line.startswith("POINTS ")), "")
        if not fields or not sizes or not types or not points_line:
            raise RuntimeError(f"Incomplete PCD header: {path}")
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise RuntimeError(f"PCD file does not contain x/y/z fields: {path}")
        if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
            raise RuntimeError(f"PCD header field metadata length mismatch: {path}")
        point_count = int(points_line.split()[1])

        offsets: Dict[str, int] = {}
        offset = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets[field] = offset
            offset += size * count
        point_step = offset

        def unpack_field(raw_point: bytes, field: str):
            idx = fields.index(field)
            size = sizes[idx]
            field_type = types[idx]
            field_offset = offsets[field]
            if field_type == "F" and size == 4:
                return struct.unpack_from("<f", raw_point, field_offset)[0]
            if field_type == "F" and size == 8:
                return struct.unpack_from("<d", raw_point, field_offset)[0]
            if field_type == "I" and size == 4:
                return float(struct.unpack_from("<i", raw_point, field_offset)[0])
            if field_type == "U" and size == 4:
                return float(struct.unpack_from("<I", raw_point, field_offset)[0])
            raise RuntimeError(f"Unsupported PCD field type for {field}: {field_type}{size}")

        if data_type == "binary":
            raw = f.read(point_count * point_step)
            if len(raw) != point_count * point_step:
                raise RuntimeError(f"PCD binary data is truncated: {path}")
            for i in range(point_count):
                raw_point = raw[i * point_step : (i + 1) * point_step]
                yield (
                    float(unpack_field(raw_point, "x")),
                    float(unpack_field(raw_point, "y")),
                    float(unpack_field(raw_point, "z")),
                )
        elif data_type == "ascii":
            x_idx = fields.index("x")
            y_idx = fields.index("y")
            z_idx = fields.index("z")
            for _ in range(point_count):
                line = f.readline().decode("ascii")
                parts = line.split()
                if len(parts) <= max(x_idx, y_idx, z_idx):
                    continue
                yield float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])
        else:
            raise RuntimeError(f"Unsupported PCD DATA type {data_type}: {path}")


def native_process_env(gtsam_lib_dir: str) -> Dict[str, str]:
    env = os.environ.copy()
    if gtsam_lib_dir:
        lib_dir = os.path.abspath(gtsam_lib_dir)
        if os.path.isdir(lib_dir):
            existing = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{lib_dir}:{existing}" if existing else lib_dir
        else:
            raise RuntimeError(f"Configured gtsam_lib_dir does not exist: {lib_dir}")
    return env


def run_mme(name: str, pcd_path: str, thr_num: int, log_dir: str) -> Tuple[StageResult, float, float]:
    start = time.time()
    log_path = os.path.join(log_dir, f"{name}.log")
    mean, std, sampled = calculate_mme_python(pcd_path, max(1, int(thr_num)))
    duration = time.time() - start
    with open(log_path, "w") as f:
        f.write(f"Python MME approximation for {pcd_path}\n")
        f.write(f"sampled_points {sampled}\n")
        f.write(f"MME mean {mean} std {std}\n")
    return StageResult(name, 0, duration, log_path, "", "ok"), mean, std


def calculate_mme_python(
    pcd_path: str,
    thr_num: int,
    radius: float = 0.3,
    min_neighbors: int = 15,
    max_samples: int = 6000,
) -> Tuple[float, float, int]:
    points = np.array(list(read_xyz_pcd(pcd_path)), dtype=np.float64)
    if points.size == 0:
        raise RuntimeError(f"Cannot calculate MME for empty cloud: {pcd_path}")
    if len(points) > max_samples:
        step = int(math.ceil(len(points) / float(max_samples)))
        sample = points[::step]
    else:
        sample = points

    cell_size = radius
    grid: Dict[Tuple[int, int, int], List[int]] = {}
    keys = np.floor(points / cell_size).astype(np.int64)
    for idx, key in enumerate(keys):
        grid.setdefault((int(key[0]), int(key[1]), int(key[2])), []).append(idx)

    entropies: List[float] = []
    radius_sq = radius * radius
    for point in sample:
        base = tuple(np.floor(point / cell_size).astype(np.int64))
        neighbor_indices: List[int] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    neighbor_indices.extend(
                        grid.get((base[0] + dx, base[1] + dy, base[2] + dz), [])
                    )
        if len(neighbor_indices) <= min_neighbors:
            entropies.append(0.0)
            continue
        candidates = points[neighbor_indices]
        diff = candidates - point
        local = candidates[np.einsum("ij,ij->i", diff, diff) <= radius_sq]
        if len(local) <= min_neighbors:
            entropies.append(0.0)
            continue
        cov = np.cov(local.T, bias=True)
        det = float(np.linalg.det((2.0 * math.pi * math.e) * cov))
        if det <= 0.0 or not math.isfinite(det):
            entropies.append(0.0)
        else:
            entropies.append(0.5 * math.log(det))

    if not entropies:
        return 0.0, 0.0, 0
    arr = np.array(entropies, dtype=np.float64)
    part_count = max(1, min(thr_num, len(arr)))
    part_means = [float(np.mean(part)) for part in np.array_split(arr, part_count) if len(part)]
    mean = float(np.mean(part_means))
    std = float(np.std(part_means, ddof=1)) if len(part_means) > 1 else 0.0
    return mean, std, len(arr)


def format_optional_metric(metric: Optional[Tuple[float, float]]) -> Tuple[str, str]:
    if metric is None:
        return "nan", "nan"
    return f"{metric[0]:.12f}", f"{metric[1]:.12f}"


def optional_metric_delta(
    before: Optional[Tuple[float, float]],
    after: Optional[Tuple[float, float]],
) -> Tuple[float, float]:
    if before is None or after is None:
        return float("nan"), float("nan")
    delta = after[0] - before[0]
    percent = 100.0 * delta / abs(before[0]) if abs(before[0]) > 1e-12 else 0.0
    return delta, percent


def optimization_verdict(
    counts: Dict[str, object],
    pgo_mme: Optional[Tuple[float, float]],
    hba_mme: Optional[Tuple[float, float]],
) -> Dict[str, object]:
    pgo_loop_count = int(counts["pgo_loop_count"])
    pgo_abs_changed = (
        float(counts["pgo_translation_delta_max"]) > 0.01
        or float(counts["pgo_rotation_delta_rad_max"]) > math.radians(1.0)
    )
    pgo_rel_changed = (
        float(counts["pgo_relative_translation_delta_m_p95"]) > 0.01
        or float(counts["pgo_relative_rotation_delta_rad_p95"]) > math.radians(1.0)
    )
    hba_abs_changed = (
        float(counts["hba_translation_delta_max"]) > 0.01
        or float(counts["hba_rotation_delta_rad_max"]) > math.radians(1.0)
    )
    hba_rel_changed = (
        float(counts["hba_relative_translation_delta_m_p95"]) > 0.01
        or float(counts["hba_relative_rotation_delta_rad_p95"]) > math.radians(1.0)
    )
    mme_delta, mme_delta_percent = optional_metric_delta(pgo_mme, hba_mme)
    hba_mme_improved = bool(hba_mme is not None and pgo_mme is not None and hba_mme[0] < pgo_mme[0])

    notes: List[str] = []
    if pgo_loop_count == 0:
        notes.append("PGO loop file is empty; native PGO ran, but there were no loop-closure factors.")
    if not pgo_abs_changed and not pgo_rel_changed:
        notes.append("PGO trajectory is effectively unchanged from the SLAM pose chain.")
    if hba_abs_changed or hba_rel_changed:
        notes.append("HBA changed the trajectory relative to PGO.")
    if hba_mme is not None and pgo_mme is not None:
        if hba_mme_improved:
            notes.append("HBA map MME improved over PGO using the lower-is-better convention.")
        else:
            notes.append("HBA map MME did not improve over PGO using the lower-is-better convention.")

    if pgo_loop_count == 0 and not pgo_abs_changed:
        overall = "pgo_no_loop_closure"
    elif pgo_abs_changed or pgo_rel_changed:
        overall = "pgo_changed_trajectory"
    else:
        overall = "pgo_computed_without_obvious_pose_change"
    if hba_abs_changed or hba_rel_changed:
        overall += "_hba_changed_trajectory"
    if hba_mme is not None and pgo_mme is not None:
        overall += "_mme_improved" if hba_mme_improved else "_mme_not_improved"

    return {
        "overall": overall,
        "pgo_has_loop_closure": pgo_loop_count > 0,
        "pgo_absolute_pose_changed": pgo_abs_changed,
        "pgo_relative_motion_changed": pgo_rel_changed,
        "hba_absolute_pose_changed": hba_abs_changed,
        "hba_relative_motion_changed": hba_rel_changed,
        "hba_mme_improved_over_pgo": hba_mme_improved,
        "hba_mme_delta": mme_delta,
        "hba_mme_delta_percent": mme_delta_percent,
        "notes": notes,
    }


def write_optimization_evaluation_yaml(
    output_root: str,
    params: Dict[str, object],
    counts: Dict[str, object],
    pgo_mme: Optional[Tuple[float, float]],
    hba_mme: Optional[Tuple[float, float]],
    paths: Dict[str, str],
) -> str:
    pgo_mme_mean, pgo_mme_std = format_optional_metric(pgo_mme)
    hba_mme_mean, hba_mme_std = format_optional_metric(hba_mme)
    verdict = optimization_verdict(counts, pgo_mme, hba_mme)

    def strip_prefix_map(prefix: str) -> Dict[str, object]:
        return {
            key[len(prefix):]: value
            for key, value in counts.items()
            if key.startswith(prefix)
        }

    report = {
        "verdict": verdict,
        "configuration": {
            "bag_path": params["bag_path"],
            "world_frame_id": params["world_frame_id"],
            "pgo_use_key_frame": params["pgo_use_key_frame"],
            "pgo_key_frame_len_thre": params["pgo_key_frame_len_thre"],
            "pgo_key_frame_ang_thre": params["pgo_key_frame_ang_thre"],
            "pgo_std": params["pgo_std_params"],
            "stitch_pcd": params["stitch_pcd"],
            "extrinsic_apply": params["slam_stitch_extrinsic_apply"],
            "extrinsic_pose_mode": params["slam_stitch_extrinsic_pose_mode"],
        },
        "frame_counts": {
            "odom_count": counts["odom_count"],
            "cloud_count": counts["cloud_count"],
            "pgo_pose_count": counts["pgo_pose_count"],
            "pgo_loop_count": counts["pgo_loop_count"],
            "hba_pose_count": counts["hba_pose_count"],
            "input_world_clouds": counts["input_world_clouds"],
            "pgo_bag_frames": counts["pgo_bag_frames"],
            "hba_bag_frames": counts["hba_bag_frames"],
        },
        "slam_trajectory": strip_prefix_map("slam_trajectory_"),
        "pgo_vs_slam": {
            "absolute_pose_delta": {
                key[len("pgo_"):]: value
                for key, value in counts.items()
                if key.startswith("pgo_translation_delta_")
                or key.startswith("pgo_rotation_delta_rad_")
            },
            "relative_motion_delta": {
                key[len("pgo_"):]: value
                for key, value in counts.items()
                if key.startswith("pgo_relative_")
                or key.startswith("pgo_path_length_delta")
            },
            "trajectory": strip_prefix_map("pgo_trajectory_"),
            "map_points": counts["pgo_map_points"],
            "mme_mean": pgo_mme_mean,
            "mme_std": pgo_mme_std,
        },
        "hba_vs_pgo": {
            "absolute_pose_delta": {
                key[len("hba_"):]: value
                for key, value in counts.items()
                if key.startswith("hba_translation_delta_")
                or key.startswith("hba_rotation_delta_rad_")
            },
            "relative_motion_delta": {
                key[len("hba_"):]: value
                for key, value in counts.items()
                if key.startswith("hba_relative_")
                or key.startswith("hba_path_length_delta")
            },
            "trajectory": strip_prefix_map("hba_trajectory_"),
            "map_points": counts["hba_map_points"],
            "mme_mean": hba_mme_mean,
            "mme_std": hba_mme_std,
        },
        "outputs": paths,
    }
    path = os.path.join(output_root, "optimization_evaluation.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(report, f, sort_keys=False, allow_unicode=False)
    return path


def write_report(
    output_root: str,
    params: Dict[str, object],
    counts: Dict[str, object],
    stages: List[StageResult],
    pgo_mme: Optional[Tuple[float, float]],
    hba_mme: Optional[Tuple[float, float]],
    paths: Dict[str, str],
) -> None:
    pgo_mme_mean, pgo_mme_std = format_optional_metric(pgo_mme)
    hba_mme_mean, hba_mme_std = format_optional_metric(hba_mme)
    verdict = optimization_verdict(counts, pgo_mme, hba_mme)
    improved = bool(verdict["hba_mme_improved_over_pgo"])
    mme_delta, mme_delta_percent = optional_metric_delta(pgo_mme, hba_mme)
    optimization_evaluation_path = write_optimization_evaluation_yaml(
        output_root, params, counts, pgo_mme, hba_mme, paths
    )

    def write_pose_delta_fields(stream, prefix: str) -> None:
        stream.write(
            f"{prefix}_translation_delta_min: "
            f"{float(counts[f'{prefix}_translation_delta_min']):.12f}\n"
        )
        stream.write(
            f"{prefix}_translation_delta_mean: "
            f"{float(counts[f'{prefix}_translation_delta_mean']):.12f}\n"
        )
        stream.write(
            f"{prefix}_translation_delta_p95: "
            f"{float(counts[f'{prefix}_translation_delta_p95']):.12f}\n"
        )
        stream.write(
            f"{prefix}_translation_delta_max: "
            f"{float(counts[f'{prefix}_translation_delta_max']):.12f}\n"
        )
        stream.write(
            f"{prefix}_rotation_delta_rad_min: "
            f"{float(counts[f'{prefix}_rotation_delta_rad_min']):.12f}\n"
        )
        stream.write(
            f"{prefix}_rotation_delta_rad_mean: "
            f"{float(counts[f'{prefix}_rotation_delta_rad_mean']):.12f}\n"
        )
        stream.write(
            f"{prefix}_rotation_delta_rad_p95: "
            f"{float(counts[f'{prefix}_rotation_delta_rad_p95']):.12f}\n"
        )
        stream.write(
            f"{prefix}_rotation_delta_rad_max: "
            f"{float(counts[f'{prefix}_rotation_delta_rad_max']):.12f}\n"
        )
        stream.write(
            f"{prefix}_translation_delta_gt_1cm: "
            f"{int(counts[f'{prefix}_translation_delta_gt_1cm'])}\n"
        )
        stream.write(
            f"{prefix}_rotation_delta_gt_1deg: "
            f"{int(counts[f'{prefix}_rotation_delta_gt_1deg'])}\n"
        )
        stream.write(
            f"{prefix}_relative_translation_delta_m_mean: "
            f"{float(counts[f'{prefix}_relative_translation_delta_m_mean']):.12f}\n"
        )
        stream.write(
            f"{prefix}_relative_translation_delta_m_p95: "
            f"{float(counts[f'{prefix}_relative_translation_delta_m_p95']):.12f}\n"
        )
        stream.write(
            f"{prefix}_relative_translation_delta_m_max: "
            f"{float(counts[f'{prefix}_relative_translation_delta_m_max']):.12f}\n"
        )
        stream.write(
            f"{prefix}_relative_rotation_delta_rad_mean: "
            f"{float(counts[f'{prefix}_relative_rotation_delta_rad_mean']):.12f}\n"
        )
        stream.write(
            f"{prefix}_relative_rotation_delta_rad_p95: "
            f"{float(counts[f'{prefix}_relative_rotation_delta_rad_p95']):.12f}\n"
        )
        stream.write(
            f"{prefix}_relative_rotation_delta_rad_max: "
            f"{float(counts[f'{prefix}_relative_rotation_delta_rad_max']):.12f}\n"
        )
        stream.write(
            f"{prefix}_path_length_delta_m: "
            f"{float(counts[f'{prefix}_path_length_delta_m']):.12f}\n"
        )
        stream.write(
            f"{prefix}_path_length_delta_percent: "
            f"{float(counts[f'{prefix}_path_length_delta_percent']):.12f}\n"
        )

    evaluation_path = os.path.join(output_root, "evaluation.txt")
    with open(evaluation_path, "w") as f:
        f.write("Spikive Guangzhou PGO + HBA evaluation\n")
        f.write("\nverdict:\n")
        f.write(f"overall: {verdict['overall']}\n")
        f.write(f"pgo_has_loop_closure: {str(verdict['pgo_has_loop_closure']).lower()}\n")
        f.write(
            f"pgo_absolute_pose_changed: "
            f"{str(verdict['pgo_absolute_pose_changed']).lower()}\n"
        )
        f.write(
            f"pgo_relative_motion_changed: "
            f"{str(verdict['pgo_relative_motion_changed']).lower()}\n"
        )
        f.write(
            f"hba_absolute_pose_changed: "
            f"{str(verdict['hba_absolute_pose_changed']).lower()}\n"
        )
        f.write(
            f"hba_relative_motion_changed: "
            f"{str(verdict['hba_relative_motion_changed']).lower()}\n"
        )
        f.write(f"hba_mme_improved_over_pgo: {str(improved).lower()}\n")
        f.write(f"hba_mme_delta: {mme_delta:.12f}\n")
        f.write(f"hba_mme_delta_percent: {mme_delta_percent:.12f}\n")
        for note in verdict["notes"]:
            f.write(f"- {note}\n")
        f.write("\ninputs:\n")
        f.write(f"bag_path: {params['bag_path']}\n")
        f.write(f"pointcloud_process_config: {params['pointcloud_process_config']}\n")
        f.write(f"cloud_topic: {params['cloud_topic']}\n")
        f.write(f"odom_topic: {params['odom_topic']}\n")
        f.write(f"pgo_input_cloud_topic: {params['pgo_input_cloud_topic']}\n")
        f.write(f"world_frame_id: {params['world_frame_id']}\n")
        f.write(f"pgo_use_key_frame: {str(params['pgo_use_key_frame']).lower()}\n")
        f.write(f"pgo_key_frame_len_thre: {params['pgo_key_frame_len_thre']}\n")
        f.write(f"pgo_key_frame_ang_thre: {params['pgo_key_frame_ang_thre']}\n")
        f.write("pgo_std:\n")
        for key, value in sorted(params["pgo_std_params"].items()):
            f.write(f"  {key}: {value}\n")
        f.write(f"stitch_pcd: {str(params['stitch_pcd']).lower()}\n")
        f.write(f"slam_stitch_extrinsic_apply: {str(params['slam_stitch_extrinsic_apply']).lower()}\n")
        f.write(f"slam_stitch_extrinsic_pose_mode: {params['slam_stitch_extrinsic_pose_mode']}\n")
        f.write(f"slam_stitch_extrinsic_r: {params['slam_stitch_extrinsic_r']}\n")
        f.write(f"slam_stitch_extrinsic_t: {params['slam_stitch_extrinsic_t']}\n")
        f.write(f"odom_count: {counts['odom_count']}\n")
        f.write(f"cloud_count: {counts['cloud_count']}\n")
        f.write(f"pgo_pose_count: {counts['pgo_pose_count']}\n")
        f.write(f"pgo_loop_count: {counts['pgo_loop_count']}\n")
        f.write(f"hba_pose_count: {counts['hba_pose_count']}\n")
        f.write(f"pgo_bag_frames: {counts['pgo_bag_frames']}\n")
        f.write(f"hba_bag_frames: {counts['hba_bag_frames']}\n")
        f.write(f"input_map_points: {counts['input_map_points']}\n")
        f.write(f"pgo_map_points: {counts['pgo_map_points']}\n")
        f.write(f"hba_map_points: {counts['hba_map_points']}\n")
        write_pose_delta_fields(f, "pgo")
        write_pose_delta_fields(f, "hba")
        f.write(f"pgo_optimized_bag: {paths['pgo_optimized_bag']}\n")
        f.write(f"hba_optimized_bag: {paths['hba_optimized_bag']}\n")
        f.write(f"pgo_mme_mean: {pgo_mme_mean}\n")
        f.write(f"pgo_mme_std: {pgo_mme_std}\n")
        f.write(f"hba_mme_mean: {hba_mme_mean}\n")
        f.write(f"hba_mme_std: {hba_mme_std}\n")
        f.write(f"hba_improved_mme: {str(improved).lower()}\n")
        f.write(f"optimization_evaluation: {optimization_evaluation_path}\n")
        f.write("\nstages:\n")
        for stage in stages:
            f.write(
                f"- {stage.name}: returncode={stage.returncode} "
                f"status={stage.status} duration_sec={stage.duration_sec:.2f} "
                f"log={stage.log_path}\n"
            )
        f.write("\noutputs:\n")
        for key, value in paths.items():
            f.write(f"- {key}: {value}\n")

    summary_path = os.path.join(output_root, "summary.yaml")
    with open(summary_path, "w") as f:
        f.write(f"bag_path: {params['bag_path']}\n")
        f.write(f"pointcloud_process_config: {params['pointcloud_process_config']}\n")
        f.write(f"cloud_topic: {params['cloud_topic']}\n")
        f.write(f"odom_topic: {params['odom_topic']}\n")
        f.write(f"pgo_input_cloud_topic: {params['pgo_input_cloud_topic']}\n")
        f.write(f"world_frame_id: {params['world_frame_id']}\n")
        f.write(f"pgo_use_key_frame: {str(params['pgo_use_key_frame']).lower()}\n")
        f.write(f"pgo_key_frame_len_thre: {params['pgo_key_frame_len_thre']}\n")
        f.write(f"pgo_key_frame_ang_thre: {params['pgo_key_frame_ang_thre']}\n")
        f.write("pgo_std:\n")
        for key, value in sorted(params["pgo_std_params"].items()):
            f.write(f"  {key}: {value}\n")
        f.write(f"stitch_pcd: {str(params['stitch_pcd']).lower()}\n")
        f.write(f"slam_stitch_extrinsic_apply: {str(params['slam_stitch_extrinsic_apply']).lower()}\n")
        f.write(f"slam_stitch_extrinsic_pose_mode: {params['slam_stitch_extrinsic_pose_mode']}\n")
        f.write(f"slam_stitch_extrinsic_r: {params['slam_stitch_extrinsic_r']}\n")
        f.write(f"slam_stitch_extrinsic_t: {params['slam_stitch_extrinsic_t']}\n")
        for key, value in counts.items():
            if isinstance(value, float):
                f.write(f"{key}: {value:.12f}\n")
            else:
                f.write(f"{key}: {value}\n")
        f.write(f"pgo_mme_mean: {pgo_mme_mean}\n")
        f.write(f"pgo_mme_std: {pgo_mme_std}\n")
        f.write(f"hba_mme_mean: {hba_mme_mean}\n")
        f.write(f"hba_mme_std: {hba_mme_std}\n")
        f.write(f"hba_improved_mme: {str(improved).lower()}\n")
        f.write(f"hba_mme_delta: {mme_delta:.12f}\n")
        f.write(f"hba_mme_delta_percent: {mme_delta_percent:.12f}\n")
        f.write(f"optimization_verdict: {verdict['overall']}\n")
        f.write(f"optimization_evaluation: {optimization_evaluation_path}\n")
        f.write(f"pgo_optimized_bag: {paths['pgo_optimized_bag']}\n")
        f.write(f"hba_optimized_bag: {paths['hba_optimized_bag']}\n")
        f.write("stages:\n")
        for stage in stages:
            f.write(
                f"  - name: {stage.name}\n"
                f"    returncode: {stage.returncode}\n"
                f"    status: {stage.status}\n"
                f"    duration_sec: {stage.duration_sec:.3f}\n"
                f"    log_path: {stage.log_path}\n"
            )
        f.write("outputs:\n")
        for key, value in paths.items():
            f.write(f"  {key}: {value}\n")


def count_lines(path: str) -> int:
    if not os.path.exists(path):
        return 0
    with open(path, "r") as f:
        return sum(1 for line in f if line.strip())


def require_file(path: str, description: str) -> None:
    if not os.path.exists(path):
        raise RuntimeError(f"Missing {description}: {path}")
    if os.path.getsize(path) == 0:
        raise RuntimeError(f"Empty {description}: {path}")


def main() -> int:
    rospy.init_node("drone_2_guangzhou_pipeline")

    project_root = project_dir()
    pipeline_config_path = private_param("config", default_pipeline_config_path())
    pipeline_config_path = resolve_relative_path(pipeline_config_path, package_dir())
    pipeline_config = merge_dicts(default_pipeline_config(), load_yaml_config(pipeline_config_path))

    pointcloud_config_path = private_param(
        "pointcloud_process_config",
        config_get(
            pipeline_config,
            "reference.pointcloud_process_config",
            default_pointcloud_process_config_path(),
        ),
    )
    pointcloud_config_path = resolve_relative_path(pointcloud_config_path, package_dir())
    pointcloud_config = load_yaml_config(pointcloud_config_path)

    default_output_root = os.path.join(project_root, "output")
    bag_path = resolved_param(
        "bag_path", pipeline_config, "input.bag_path", config_get(pointcloud_config, "bag", "")
    )
    bag_path = os.path.abspath(resolve_relative_path(bag_path, project_root))
    output_root = os.path.abspath(resolve_relative_path(
        resolved_param("output_root", pipeline_config, "output.root", default_output_root),
        project_root,
    ))
    output_config = config_get(pipeline_config, "output", {})
    pgo_config = config_get(pipeline_config, "pgo", {})
    hba_config = config_get(pipeline_config, "hba", {})
    evaluation_config = config_get(pipeline_config, "evaluation", {})
    runtime_config = config_get(pipeline_config, "runtime", {})
    extrinsic_config = config_get(pipeline_config, "extrinsic", {})
    cloud_topic = resolved_param(
        "cloud_topic",
        pipeline_config,
        "input.cloud_topic",
        "/cloud_registered_body",
        pointcloud_config,
        "topics.cloud",
    )
    odom_topic = resolved_param(
        "odom_topic",
        pipeline_config,
        "input.odom_topic",
        "/drone_2_visual_slam/odom",
        pointcloud_config,
        "topics.odom",
    )
    pgo_input_cloud_topic = resolved_param(
        "pgo_input_cloud_topic",
        pipeline_config,
        "pgo.input_cloud_topic",
        "/cloud_registered_body",
    )
    pgo_input_odom_topic = resolved_param(
        "pgo_input_odom_topic",
        pipeline_config,
        "pgo.input_odom_topic",
        "/Odometry_pipeline_input",
    )
    pgo_save_name = resolved_param("pgo_save_name", pipeline_config, "pgo.save_name", "pgo")
    use_loop = param_bool(resolved_param("use_loop", pipeline_config, "pgo.use_loop", True))
    use_gps = param_bool(resolved_param("use_gps", pipeline_config, "pgo.use_gps", False))
    pgo_use_key_frame = param_bool(
        resolved_param("pgo_use_key_frame", pipeline_config, "pgo.use_key_frame", False)
    )
    pgo_key_frame_len_thre = float(
        resolved_param("pgo_key_frame_len_thre", pipeline_config, "pgo.key_frame_len_thre", 1.0)
    )
    pgo_key_frame_ang_thre = float(
        resolved_param("pgo_key_frame_ang_thre", pipeline_config, "pgo.key_frame_ang_thre", 0.1)
    )
    map_resolution = float(
        resolved_param(
            "map_resolution",
            pipeline_config,
            "output.map_resolution",
            0.0,
            pointcloud_config,
            "processing.voxel_size",
        )
    )
    hba_total_layer_num = int(
        resolved_param("hba_total_layer_num", pipeline_config, "hba.total_layer_num", 3)
    )
    hba_thread_num = int(resolved_param("hba_thread_num", pipeline_config, "hba.thread_num", 4))
    mme_thr_num = int(resolved_param("mme_thr_num", pipeline_config, "evaluation.mme_thr_num", 4))
    pgo_std_params = resolve_pgo_std_params(pipeline_config)
    write_stage_local_clouds = param_bool(
        resolved_param("write_stage_local_clouds", pipeline_config, "output.write_stage_local_clouds", True)
    )
    write_optimized_frame_clouds = param_bool(
        resolved_param(
            "write_optimized_frame_clouds",
            pipeline_config,
            "output.write_optimized_frame_clouds",
            True,
        )
    )
    pgo_local_cloud_topic = pgo_input_cloud_topic
    stitch_pcd = param_bool(
        resolved_param("stitch_pcd", pipeline_config, "output.stitch_pcd", True)
    )
    run_native_pgo = param_bool(
        resolved_param("run_native_pgo", pipeline_config, "pgo.run_native", True)
    )
    run_native_hba = param_bool(
        resolved_param("run_native_hba", pipeline_config, "hba.run_native", True)
    )
    default_gtsam_lib_dir = resolved_param(
        "gtsam_lib_dir",
        pipeline_config,
        "runtime.gtsam_lib_dir",
        "/workspace/catkin_ws/third_party/install/lib",
    )
    pgo_gtsam_lib_dir = resolved_param(
        "pgo_gtsam_lib_dir",
        pipeline_config,
        "runtime.pgo_gtsam_lib_dir",
        default_gtsam_lib_dir,
    )
    hba_gtsam_lib_dir = resolved_param(
        "hba_gtsam_lib_dir",
        pipeline_config,
        "runtime.hba_gtsam_lib_dir",
        default_gtsam_lib_dir,
    )

    extrinsic_apply = param_bool(
        resolved_param(
            "slam_stitch_extrinsic_apply",
            pipeline_config,
            "extrinsic.apply",
            True,
            pointcloud_config,
            "flight_controller_extrinsic.apply",
        )
    )
    extrinsic_r = resolved_param(
        "slam_stitch_extrinsic_r",
        pipeline_config,
        "extrinsic.R",
        DEFAULT_SLAM_EXTRINSIC_R,
        pointcloud_config,
        "flight_controller_extrinsic.R",
    )
    extrinsic_t = resolved_param(
        "slam_stitch_extrinsic_t",
        pipeline_config,
        "extrinsic.T",
        DEFAULT_SLAM_EXTRINSIC_T,
        pointcloud_config,
        "flight_controller_extrinsic.T",
    )
    extrinsic_pose_mode = str(
        resolved_param(
            "slam_stitch_extrinsic_pose_mode",
            pipeline_config,
            "extrinsic.pose_mode",
            "right_multiply",
        )
    )
    local_extrinsic = make_local_extrinsic(extrinsic_apply, extrinsic_r, extrinsic_t)

    if not os.path.exists(bag_path):
        raise RuntimeError(f"bag_path does not exist: {bag_path}")

    paths = ensure_clean_output_root(output_root)
    input_name = bag_stem(bag_path)
    pose_input_path = os.path.join(paths["pgo_input"], f"{input_name}_pose.txt")
    pgo_bag_path = os.path.join(paths["pgo_input"], f"{input_name}_cloud.bag")
    pgo_pose_path = os.path.join(paths["pgo_output"], f"{pgo_save_name}_pose.txt")
    pgo_loop_path = os.path.join(paths["pgo_output"], f"{pgo_save_name}_loop.txt")
    input_world_bag_path = os.path.join(paths["input"], f"{input_name}_world.bag")
    input_map_path = os.path.join(paths["input"], f"{input_name}_map.pcd")
    pgo_map_path = os.path.join(paths["pgo_output"], f"{pgo_save_name}_map.pcd")
    pgo_optimized_bag_path = os.path.join(paths["pgo_output"], f"{pgo_save_name}_optimized.bag")
    hba_map_path = os.path.join(paths["hba"], "hba_map.pcd")
    hba_optimized_bag_path = os.path.join(paths["hba"], "hba_optimized.bag")

    rospy.loginfo("Output root: %s", paths["root"])
    odom_count, cloud_count = export_pose_file(
        bag_path,
        odom_topic,
        cloud_topic,
        pose_input_path,
        local_extrinsic,
        extrinsic_pose_mode,
    )
    pgo_input_odom_count, pgo_input_cloud_count = write_pgo_input_bag(
        bag_path,
        pgo_bag_path,
        cloud_topic,
        odom_topic,
        pgo_input_cloud_topic,
        pgo_input_odom_topic,
    )
    rospy.loginfo(
        "Prepared PGO input bag with %d odom messages and %d local clouds",
        pgo_input_odom_count,
        pgo_input_cloud_count,
    )

    rospack = rospkg.RosPack()
    pgo_config_path = os.path.join(rospack.get_path("lsdc_pgo"), "config", "config_loop.yaml")
    configure_pgo_params(
        pgo_config_path,
        paths["pgo_input"],
        paths["pgo_output"],
        input_name,
        pgo_save_name,
        pgo_input_cloud_topic,
        use_loop,
        use_gps,
        pgo_use_key_frame,
        pgo_key_frame_len_thre,
        pgo_key_frame_ang_thre,
        map_resolution,
        pgo_std_params,
    )

    stages: List[StageResult] = []
    slam_poses = read_pgo_pose(pose_input_path)
    world_frame_id = str(
        resolved_param("world_frame_id", pipeline_config, "output.world_frame_id", "world")
    )

    slam_bag_stats = write_optimized_stage_bag(
        pgo_bag_path,
        input_world_bag_path,
        pgo_input_cloud_topic,
        slam_poses,
        "/spikive_pipeline/input/cloud_world",
        "/spikive_pipeline/input/odom",
        "/spikive_pipeline/input/map",
        world_frame_id,
        False,
        write_optimized_frame_clouds,
        stitch_pcd,
        input_map_path,
        map_resolution,
    )
    require_file(input_world_bag_path, "input world bag")
    if stitch_pcd:
        require_file(input_map_path, "input stitched map")
    rospy.loginfo(
        "Wrote input world bag with %d world cloud frames: %s",
        slam_bag_stats.optimized_clouds_written,
        input_world_bag_path,
    )

    native_pgo_env = native_process_env(pgo_gtsam_lib_dir)
    if not run_native_pgo:
        raise RuntimeError("run_native_pgo is false; refusing to continue without native PGO.")
    pgo_stage = run_logged(
        "pgo_std_loop",
        ["rosrun", "lsdc_pgo", "std_loop"],
        paths["logs"],
        check=False,
        env=native_pgo_env,
    )
    stages.append(pgo_stage)
    native_pgo_ok = (
        pgo_stage.returncode == 0
        and os.path.exists(pgo_pose_path)
        and os.path.getsize(pgo_pose_path) > 0
    )
    if not native_pgo_ok:
        raise RuntimeError(
            f"Native PGO did not produce a valid pose output. See {pgo_stage.log_path}"
        )
    require_file(pgo_pose_path, "PGO pose output")
    pgo_poses = read_pgo_pose(pgo_pose_path)
    if len(pgo_poses) != len(slam_poses):
        rospy.loginfo(
            "Expanding %d PGO keyframe poses to %d full-resolution poses",
            len(pgo_poses),
            len(slam_poses),
        )
        pgo_keyframe_pose_path = os.path.join(paths["pgo_output"], f"{pgo_save_name}_keyframe_pose.txt")
        shutil.copyfile(pgo_pose_path, pgo_keyframe_pose_path)
        pgo_poses = expand_keyframe_poses_to_full_frames(pgo_poses, slam_poses)
        write_pgo_pose_file(pgo_pose_path, pgo_poses)
    pgo_delta_stats = pose_delta_stats(slam_poses, pgo_poses)
    slam_trajectory_stats = trajectory_stats(slam_poses)
    pgo_trajectory_stats = trajectory_stats(pgo_poses)
    pgo_relative_stats = relative_motion_delta_stats(slam_poses, pgo_poses)

    pgo_bag_stats = write_optimized_stage_bag(
        pgo_bag_path,
        pgo_optimized_bag_path,
        pgo_input_cloud_topic,
        pgo_poses,
        "/spikive_pipeline/pgo/cloud_world",
        "/spikive_pipeline/pgo/odom",
        "/spikive_pipeline/pgo/map",
        world_frame_id,
        True,
        write_optimized_frame_clouds,
        stitch_pcd,
        pgo_map_path,
        map_resolution,
    )
    require_file(pgo_optimized_bag_path, "PGO optimized bag")
    if stitch_pcd:
        require_file(pgo_map_path, "PGO stitched map")
    rospy.loginfo(
        "Wrote PGO optimized bag with %d optimized cloud frames: %s",
        pgo_bag_stats.optimized_clouds_written,
        pgo_optimized_bag_path,
    )

    hba_pose_count = export_hba_dataset(
        pgo_optimized_bag_path,
        pgo_local_cloud_topic,
        pgo_poses,
        paths["hba"],
        paths["hba_pcd"],
    )
    write_hba_pose(os.path.join(paths["hba"], "pose.json"), pgo_poses)
    shutil.copyfile(
        os.path.join(paths["hba"], "pose.json"),
        os.path.join(paths["hba"], "pose_pgo.json"),
    )
    rospy.loginfo("Prepared HBA dataset with %d frames", hba_pose_count)

    pgo_mme: Optional[Tuple[float, float]] = None
    if stitch_pcd:
        pgo_mme_result, pgo_mme_mean, pgo_mme_std = run_mme(
            "mme_pgo", pgo_map_path, mme_thr_num, paths["logs"]
        )
        stages.append(pgo_mme_result)
        pgo_mme = (pgo_mme_mean, pgo_mme_std)

    native_hba_env = native_process_env(hba_gtsam_lib_dir)
    if not run_native_hba:
        raise RuntimeError("run_native_hba is false; refusing to continue without native HBA.")
    hba_stage = run_logged(
        "hba",
        [
            "rosrun",
            "hba",
            "hba",
            f"_data_path:={os.path.abspath(paths['hba'])}/",
            f"_total_layer_num:={int(hba_total_layer_num)}",
            "_pcd_name_fill_num:=0",
            f"_thread_num:={int(hba_thread_num)}",
        ],
        paths["logs"],
        check=False,
        env=native_hba_env,
    )
    stages.append(hba_stage)
    if hba_stage.returncode != 0:
        raise RuntimeError(f"Native HBA failed. See {hba_stage.log_path}")
    require_file(os.path.join(paths["hba"], "pose.json"), "HBA pose output")

    hba_relative_pose_path = os.path.join(paths["hba"], "pose_hba_relative.json")
    hba_world_pose_path = os.path.join(paths["hba"], "pose_hba.json")
    shutil.copyfile(os.path.join(paths["hba"], "pose.json"), hba_relative_pose_path)
    hba_relative_poses = hba_poses_with_stamps(
        hba_relative_pose_path,
        [pose.stamp for pose in pgo_poses],
    )
    hba_poses = anchor_hba_poses_to_world(hba_relative_poses, pgo_poses)
    hba_delta_stats = pose_delta_stats(pgo_poses, hba_poses)
    hba_trajectory_stats = trajectory_stats(hba_poses)
    hba_relative_stats = relative_motion_delta_stats(pgo_poses, hba_poses)
    write_hba_pose(hba_world_pose_path, hba_poses)
    write_pgo_pose_file(
        os.path.join(paths["hba"], "pose_hba_world.txt"),
        hba_poses,
    )

    hba_bag_stats = write_optimized_stage_bag(
        pgo_optimized_bag_path,
        hba_optimized_bag_path,
        pgo_local_cloud_topic,
        hba_poses,
        "/spikive_pipeline/hba/cloud_world",
        "/spikive_pipeline/hba/odom",
        "/spikive_pipeline/hba/map",
        world_frame_id,
        write_stage_local_clouds,
        write_optimized_frame_clouds,
        stitch_pcd,
        hba_map_path,
        map_resolution,
    )
    require_file(hba_optimized_bag_path, "HBA optimized bag")
    if stitch_pcd:
        require_file(hba_map_path, "HBA stitched map")
    rospy.loginfo(
        "Wrote HBA optimized bag with %d optimized cloud frames: %s",
        hba_bag_stats.optimized_clouds_written,
        hba_optimized_bag_path,
    )

    hba_mme: Optional[Tuple[float, float]] = None
    if stitch_pcd:
        hba_mme_result, hba_mme_mean, hba_mme_std = run_mme(
            "mme_hba", hba_map_path, mme_thr_num, paths["logs"]
        )
        stages.append(hba_mme_result)
        hba_mme = (hba_mme_mean, hba_mme_std)

    counts = {
        "odom_count": odom_count,
        "cloud_count": cloud_count,
        "pgo_input_odom_count": pgo_input_odom_count,
        "pgo_input_cloud_count": pgo_input_cloud_count,
        "pgo_pose_count": len(pgo_poses),
        "pgo_loop_count": count_lines(pgo_loop_path),
        "hba_pose_count": hba_pose_count,
        "pgo_bag_frames": pgo_bag_stats.frames_written,
        "hba_bag_frames": hba_bag_stats.frames_written,
        "input_world_clouds": slam_bag_stats.optimized_clouds_written,
        "pgo_optimized_clouds": pgo_bag_stats.optimized_clouds_written,
        "hba_optimized_clouds": hba_bag_stats.optimized_clouds_written,
        "input_map_points": slam_bag_stats.map_written,
        "pgo_map_points": pgo_bag_stats.map_written,
        "hba_map_points": hba_bag_stats.map_written,
    }
    counts.update({f"slam_trajectory_{key}": value for key, value in slam_trajectory_stats.items()})
    counts.update({f"pgo_trajectory_{key}": value for key, value in pgo_trajectory_stats.items()})
    counts.update({f"hba_trajectory_{key}": value for key, value in hba_trajectory_stats.items()})
    counts.update({f"pgo_{key}": value for key, value in pgo_delta_stats.items()})
    counts.update({f"pgo_{key}": value for key, value in pgo_relative_stats.items()})
    counts.update({f"hba_{key}": value for key, value in hba_delta_stats.items()})
    counts.update({f"hba_{key}": value for key, value in hba_relative_stats.items()})
    output_paths = {
        "input_world_bag": input_world_bag_path,
        "input_map": input_map_path,
        "pgo_input_bag": pgo_bag_path,
        "pgo_pose": pgo_pose_path,
        "pgo_loop": pgo_loop_path,
        "pgo_optimized_bag": pgo_optimized_bag_path,
        "pgo_map": pgo_map_path,
        "hba_pose_pgo": os.path.join(paths["hba"], "pose_pgo.json"),
        "hba_pose_hba_relative": os.path.join(paths["hba"], "pose_hba_relative.json"),
        "hba_pose_hba": os.path.join(paths["hba"], "pose_hba.json"),
        "hba_pose_hba_world_txt": os.path.join(paths["hba"], "pose_hba_world.txt"),
        "hba_optimized_bag": hba_optimized_bag_path,
        "hba_map": hba_map_path,
        "evaluation": os.path.join(paths["root"], "evaluation.txt"),
        "optimization_evaluation": os.path.join(paths["root"], "optimization_evaluation.yaml"),
        "summary": os.path.join(paths["root"], "summary.yaml"),
    }
    write_report(
        paths["root"],
        {
            "bag_path": bag_path,
            "pointcloud_process_config": pointcloud_config_path,
            "cloud_topic": cloud_topic,
            "odom_topic": odom_topic,
            "pgo_input_cloud_topic": pgo_input_cloud_topic,
            "world_frame_id": world_frame_id,
            "pgo_use_key_frame": pgo_use_key_frame,
            "pgo_key_frame_len_thre": pgo_key_frame_len_thre,
            "pgo_key_frame_ang_thre": pgo_key_frame_ang_thre,
            "pgo_std_params": pgo_std_params,
            "stitch_pcd": stitch_pcd,
            "slam_stitch_extrinsic_apply": local_extrinsic.apply,
            "slam_stitch_extrinsic_pose_mode": extrinsic_pose_mode,
            "slam_stitch_extrinsic_r": [v for row in local_extrinsic.rotation for v in row],
            "slam_stitch_extrinsic_t": list(local_extrinsic.translation),
        },
        counts,
        stages,
        pgo_mme,
        hba_mme,
        output_paths,
    )

    rospy.loginfo("Pipeline complete. Summary: %s", output_paths["summary"])
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        rospy.logerr("Pipeline failed: %s", exc)
        raise
