#!/usr/bin/env python3
"""Offline LAS ground-truth evaluation for PointCloud-PGOBA outputs.

This tool is intentionally outside the existing SLAM/PGO/HBA pipeline. It only
reads generated outputs plus a ground-truth LAS file, then writes comparison
reports. It must not be used as an optimization input.
"""

import argparse
import csv
import itertools
import math
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.spatial import cKDTree


DEFAULT_TARGETS = {
    "input": {
        "map": "input/drone_2_gld_indexed_map.pcd",
        "bag": "input/drone_2_gld_indexed_world.bag",
        "topic": "/spikive_pipeline/input/cloud_world",
    },
    "pgo": {
        "map": "pgo_output/pgo_map.pcd",
        "bag": "pgo_output/pgo_optimized.bag",
        "topic": "/spikive_pipeline/pgo/cloud_world",
    },
    "hba": {
        "map": "hba/hba_map.pcd",
        "bag": "hba/hba_optimized.bag",
        "topic": "/spikive_pipeline/hba/cloud_world",
    },
}

POINTFIELD_TO_DTYPE = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}


@dataclass
class LoadedTarget:
    case_name: str
    target_name: str
    root: Path
    map_path: Path
    bag_path: Optional[Path]
    topic: str


def project_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def resolve_path(value: str, base: Path) -> Path:
    path = Path(os.path.expanduser(str(value)))
    if path.is_absolute():
        return path
    return (base / path).resolve()


def load_yaml(path: Path) -> Dict:
    with path.open("r") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return data


def dump_yaml(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def sample_points(points: np.ndarray, max_points: int, seed: int = 7) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(points), size=max_points, replace=False)
    return points[np.sort(idx)]


def finite_xyz(points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 3).astype(np.float64)
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return points[np.all(np.isfinite(points), axis=1)]


def voxel_downsample(points: np.ndarray, voxel_size: float, max_points: int = 0, seed: int = 7) -> np.ndarray:
    points = finite_xyz(points)
    if len(points) == 0:
        return points
    if voxel_size and voxel_size > 0.0:
        keys = np.floor(points / float(voxel_size)).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(idx)]
    return sample_points(points, int(max_points), seed=seed)


def parse_pcd_header(f) -> Tuple[Dict[str, List[str]], bytes]:
    meta: Dict[str, List[str]] = {}
    header_lines: List[bytes] = []
    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("PCD header ended before DATA line")
        header_lines.append(line)
        stripped = line.decode("ascii", errors="replace").strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        meta[parts[0].upper()] = parts[1:]
        if parts[0].upper() == "DATA":
            return meta, b"".join(header_lines)


def pcd_dtype(meta: Dict[str, List[str]]) -> np.dtype:
    fields = meta.get("FIELDS", [])
    sizes = [int(v) for v in meta.get("SIZE", [])]
    types = meta.get("TYPE", [])
    counts = [int(v) for v in meta.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise RuntimeError("Invalid PCD field metadata")
    offsets: Dict[str, int] = {}
    offset = 0
    formats = []
    names = []
    dtype_offsets = []
    for field, size, kind, count in zip(fields, sizes, types, counts):
        offsets[field] = offset
        if field in ("x", "y", "z"):
            if kind != "F" or size not in (4, 8) or count != 1:
                raise RuntimeError(f"Unsupported PCD xyz field type: {field} {kind}{size} count={count}")
            names.append(field)
            formats.append("<f4" if size == 4 else "<f8")
            dtype_offsets.append(offset)
        offset += size * count
    if not all(field in offsets for field in ("x", "y", "z")):
        raise RuntimeError("PCD file does not contain x/y/z fields")
    return np.dtype({"names": names, "formats": formats, "offsets": dtype_offsets, "itemsize": offset})


def read_pcd_points(path: Path, max_points: int = 0, voxel_size: float = 0.0, seed: int = 7) -> np.ndarray:
    with path.open("rb") as f:
        meta, _ = parse_pcd_header(f)
        points = int((meta.get("POINTS") or meta.get("WIDTH") or ["0"])[0])
        data_type = (meta.get("DATA") or [""])[0].lower()
        if data_type == "binary":
            dtype = pcd_dtype(meta)
            arr = np.frombuffer(f.read(dtype.itemsize * points), dtype=dtype, count=points)
            xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64, copy=False)
        elif data_type == "ascii":
            rows = []
            stride = max(1, int(math.ceil(points / max_points))) if max_points and points > max_points else 1
            for i, line in enumerate(f):
                if i % stride != 0:
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
            xyz = np.asarray(rows, dtype=np.float64)
        else:
            raise RuntimeError(f"Unsupported PCD DATA type '{data_type}' in {path}")
    return voxel_downsample(xyz, voxel_size, max_points=max_points, seed=seed)


def write_pcd_xyz(path: Path, points: np.ndarray) -> None:
    points = finite_xyz(points).astype(np.float32, copy=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.tobytes(order="C"))


def read_las_points(path: Path, max_points: int = 0, voxel_size: float = 0.0, seed: int = 7) -> np.ndarray:
    if path.suffix.lower() == ".laz":
        return read_las_with_laspy(path, max_points=max_points, voxel_size=voxel_size, seed=seed)

    with path.open("rb") as f:
        header = f.read(375)
        if header[0:4] != b"LASF":
            raise RuntimeError(f"Not a LAS file: {path}")
        version_minor = header[25]
        point_offset = struct.unpack_from("<I", header, 96)[0]
        point_format = header[104] & 0x3F
        record_len = struct.unpack_from("<H", header, 105)[0]
        point_count = struct.unpack_from("<I", header, 107)[0]
        if point_count == 0 and version_minor >= 4 and len(header) >= 255:
            point_count = struct.unpack_from("<Q", header, 247)[0]
        scale = np.asarray(struct.unpack_from("<ddd", header, 131), dtype=np.float64)
        offset = np.asarray(struct.unpack_from("<ddd", header, 155), dtype=np.float64)
        if point_format > 10:
            raise RuntimeError(f"Unsupported LAS point format {point_format}")
        stride = max(1, int(math.ceil(point_count / max_points))) if max_points and point_count > max_points else 1
        dtype = np.dtype(
            {
                "names": ["X", "Y", "Z"],
                "formats": ["<i4", "<i4", "<i4"],
                "offsets": [0, 4, 8],
                "itemsize": record_len,
            }
        )
        f.seek(point_offset)
        chunks = []
        chunk_records = 1_000_000
        read_records = 0
        while read_records < point_count:
            count = min(chunk_records, point_count - read_records)
            blob = f.read(record_len * count)
            if not blob:
                break
            arr = np.frombuffer(blob, dtype=dtype, count=count)
            local_idx = np.arange(read_records, read_records + count)
            arr = arr[(local_idx % stride) == 0]
            if len(arr):
                xyz = np.column_stack([arr["X"], arr["Y"], arr["Z"]]).astype(np.float64)
                xyz = xyz * scale + offset
                chunks.append(xyz)
            read_records += count
    if not chunks:
        raise RuntimeError(f"No LAS points read from {path}")
    return voxel_downsample(np.vstack(chunks), voxel_size, max_points=max_points, seed=seed)


def read_las_with_laspy(path: Path, max_points: int = 0, voxel_size: float = 0.0, seed: int = 7) -> np.ndarray:
    try:
        import laspy  # type: ignore
    except ImportError as exc:
        raise RuntimeError("Reading LAZ requires laspy/lazrs. Use uncompressed LAS or install laspy.") from exc
    las = laspy.read(str(path))
    point_count = len(las.x)
    stride = max(1, int(math.ceil(point_count / max_points))) if max_points and point_count > max_points else 1
    xyz = np.column_stack([las.x[::stride], las.y[::stride], las.z[::stride]]).astype(np.float64)
    return voxel_downsample(xyz, voxel_size, max_points=max_points, seed=seed)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = finite_xyz(points)
    if len(points) == 0:
        return points
    return points @ transform[:3, :3].T + transform[:3, 3]


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = finite_xyz(source)
    target = finite_xyz(target)
    if len(source) != len(target) or len(source) < 3:
        raise RuntimeError("best_fit_transform requires equal point counts >= 3")
    src_centroid = source.mean(axis=0)
    tgt_centroid = target.mean(axis=0)
    src_centered = source - src_centroid
    tgt_centered = target - tgt_centroid
    h = src_centered.T @ tgt_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = tgt_centroid - r @ src_centroid
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r
    transform[:3, 3] = t
    return transform


def pca_axes(points: np.ndarray) -> np.ndarray:
    centered = finite_xyz(points) - np.mean(points, axis=0)
    cov = np.cov(centered.T)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    axes = vecs[:, order]
    if np.linalg.det(axes) < 0:
        axes[:, -1] *= -1
    return axes


def initial_transforms(source: np.ndarray, target: np.ndarray) -> List[np.ndarray]:
    src_centroid = source.mean(axis=0)
    tgt_centroid = target.mean(axis=0)
    src_axes = pca_axes(source)
    tgt_axes = pca_axes(target)
    transforms = []
    identity = np.eye(4, dtype=np.float64)
    identity[:3, 3] = tgt_centroid - src_centroid
    transforms.append(identity)

    for perm in itertools.permutations(range(3)):
        p = np.eye(3)[:, perm]
        for signs in itertools.product([-1.0, 1.0], repeat=3):
            s = np.diag(signs)
            r = tgt_axes @ p @ s @ src_axes.T
            if np.linalg.det(r) < 0:
                continue
            t = tgt_centroid - r @ src_centroid
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = r
            transform[:3, 3] = t
            transforms.append(transform)
    return transforms


def icp(
    source: np.ndarray,
    target: np.ndarray,
    init_transform: np.ndarray,
    max_iterations: int,
    max_correspondence_distance: float,
    trim_fraction: float,
    min_pairs: int,
    target_tree: Optional[cKDTree] = None,
) -> Tuple[np.ndarray, Dict]:
    transform = init_transform.copy()
    tree = target_tree or cKDTree(target)
    last_rmse = None
    history = []
    for iteration in range(max_iterations):
        moved = transform_points(source, transform)
        distances, indices = tree.query(moved, k=1)
        mask = np.isfinite(distances)
        if max_correspondence_distance and max_correspondence_distance > 0:
            mask &= distances <= max_correspondence_distance
        valid = np.where(mask)[0]
        if len(valid) < min_pairs:
            valid = np.argsort(distances)[: max(min_pairs, min(len(distances), int(len(distances) * trim_fraction)))]
        if trim_fraction < 1.0 and len(valid) > min_pairs:
            keep = max(min_pairs, int(len(valid) * trim_fraction))
            valid = valid[np.argsort(distances[valid])[:keep]]
        if len(valid) < 3:
            break
        delta = best_fit_transform(moved[valid], target[indices[valid]])
        transform = delta @ transform
        rmse = float(np.sqrt(np.mean(np.square(distances[valid]))))
        p95 = float(np.percentile(distances[valid], 95))
        history.append({"iteration": iteration + 1, "pairs": int(len(valid)), "rmse": rmse, "p95": p95})
        if last_rmse is not None and abs(last_rmse - rmse) < 1e-5:
            break
        last_rmse = rmse
    return transform, {"iterations": len(history), "history": history, "rmse": last_rmse}


def align_to_groundtruth(source: np.ndarray, groundtruth: np.ndarray, cfg: Dict) -> Tuple[np.ndarray, Dict]:
    align_source = voxel_downsample(
        source,
        float(cfg.get("source_voxel_size", 0.25)),
        int(cfg.get("source_max_points", 150000)),
        seed=11,
    )
    align_gt = voxel_downsample(
        groundtruth,
        float(cfg.get("groundtruth_voxel_size", 0.25)),
        int(cfg.get("groundtruth_max_points", 250000)),
        seed=13,
    )
    if len(align_source) < 20 or len(align_gt) < 20:
        raise RuntimeError("Not enough points for alignment")

    init_source = sample_points(align_source, int(cfg.get("init_source_max_points", 20000)), seed=17)
    init_gt = sample_points(align_gt, int(cfg.get("init_groundtruth_max_points", 60000)), seed=19)
    init_tree = cKDTree(init_gt)
    candidates = initial_transforms(init_source, init_gt)
    quick = []
    for transform in candidates:
        cand_transform, cand_info = icp(
            init_source,
            init_gt,
            transform,
            max_iterations=int(cfg.get("init_iterations", 8)),
            max_correspondence_distance=float(cfg.get("init_max_correspondence_distance", 10.0)),
            trim_fraction=float(cfg.get("trim_fraction", 0.8)),
            min_pairs=int(cfg.get("min_pairs", 200)),
            target_tree=init_tree,
        )
        moved = transform_points(init_source, cand_transform)
        distances, _ = init_tree.query(moved, k=1)
        score = float(np.percentile(distances, 90))
        quick.append((score, cand_transform, cand_info))
    quick.sort(key=lambda item: item[0])
    best_init = quick[0][1]

    final_tree = cKDTree(align_gt)
    final_transform, final_info = icp(
        align_source,
        align_gt,
        best_init,
        max_iterations=int(cfg.get("max_iterations", 40)),
        max_correspondence_distance=float(cfg.get("max_correspondence_distance", 5.0)),
        trim_fraction=float(cfg.get("trim_fraction", 0.85)),
        min_pairs=int(cfg.get("min_pairs", 500)),
        target_tree=final_tree,
    )
    return final_transform, {
        "source_points": int(len(align_source)),
        "groundtruth_points": int(len(align_gt)),
        "candidate_count": int(len(candidates)),
        "best_initial_p90": float(quick[0][0]),
        "final": final_info,
    }


def distance_stats(distances: np.ndarray, thresholds: Sequence[float]) -> Dict:
    distances = np.asarray(distances, dtype=np.float64)
    distances = distances[np.isfinite(distances)]
    if len(distances) == 0:
        return {"count": 0}
    stats = {
        "count": int(len(distances)),
        "mean": float(np.mean(distances)),
        "rmse": float(np.sqrt(np.mean(np.square(distances)))),
        "p50": float(np.percentile(distances, 50)),
        "p90": float(np.percentile(distances, 90)),
        "p95": float(np.percentile(distances, 95)),
        "p99": float(np.percentile(distances, 99)),
        "max": float(np.max(distances)),
    }
    for threshold in thresholds:
        key = f"within_{threshold:.3f}m".replace(".", "p")
        stats[key] = float(np.mean(distances <= threshold))
    return stats


def map_metrics(
    transformed_source: np.ndarray,
    groundtruth: np.ndarray,
    cfg: Dict,
    thresholds: Sequence[float],
) -> Dict:
    src_eval = voxel_downsample(
        transformed_source,
        float(cfg.get("metric_source_voxel_size", 0.10)),
        int(cfg.get("metric_source_max_points", 500000)),
        seed=23,
    )
    gt_eval = voxel_downsample(
        groundtruth,
        float(cfg.get("metric_groundtruth_voxel_size", 0.10)),
        int(cfg.get("metric_groundtruth_max_points", 500000)),
        seed=29,
    )
    gt_tree = cKDTree(gt_eval)
    src_to_gt, _ = gt_tree.query(src_eval, k=1)
    src_tree = cKDTree(src_eval)
    gt_to_src, _ = src_tree.query(gt_eval, k=1)
    return {
        "source_points": int(len(src_eval)),
        "groundtruth_points": int(len(gt_eval)),
        "source_to_groundtruth": distance_stats(src_to_gt, thresholds),
        "groundtruth_to_source": distance_stats(gt_to_src, thresholds),
    }


def transform_to_yaml(transform: np.ndarray) -> Dict:
    return {
        "matrix_row_major": [[float(v) for v in row] for row in transform],
        "translation": [float(v) for v in transform[:3, 3]],
        "rotation": [[float(v) for v in row] for row in transform[:3, :3]],
    }


def import_rosbag():
    try:
        import rosbag  # type: ignore

        return rosbag, None
    except Exception as exc:
        return None, str(exc)


def pointcloud2_to_xyz(msg) -> np.ndarray:
    field_map = {field.name: field for field in msg.fields}
    if not all(name in field_map for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    endian = ">" if msg.is_bigendian else "<"
    names = []
    formats = []
    offsets = []
    for name in ("x", "y", "z"):
        field = field_map[name]
        base = POINTFIELD_TO_DTYPE.get(field.datatype)
        if base is None or field.count != 1:
            raise RuntimeError(f"Unsupported PointCloud2 field {name}: datatype={field.datatype}, count={field.count}")
        names.append(name)
        formats.append(endian + base)
        offsets.append(field.offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    count = int(msg.width * msg.height)
    arr = np.frombuffer(msg.data, dtype=dtype, count=count)
    return finite_xyz(np.column_stack([arr["x"], arr["y"], arr["z"]]))


def load_start_segment_points(target: LoadedTarget, frame_cfg: Dict) -> Tuple[Optional[np.ndarray], Optional[str]]:
    rosbag, error = import_rosbag()
    if rosbag is None:
        return None, f"rosbag unavailable: {error}"
    if target.bag_path is None or not target.bag_path.exists():
        return None, f"bag missing: {target.bag_path}"
    max_frames = int(frame_cfg.get("start_segment_max_frames", 100))
    duration = float(frame_cfg.get("start_segment_duration_sec", 10.0))
    per_frame_max = int(frame_cfg.get("start_segment_points_per_frame", 10000))
    chunks = []
    start_stamp = None
    frame_count = 0
    with rosbag.Bag(str(target.bag_path), "r") as bag:
        for _, msg, t in bag.read_messages(topics=[target.topic]):
            stamp = msg.header.stamp.to_sec() if hasattr(msg, "header") else t.to_sec()
            if start_stamp is None:
                start_stamp = stamp
            if frame_count >= max_frames or stamp - start_stamp > duration:
                break
            pts = sample_points(pointcloud2_to_xyz(msg), per_frame_max, seed=101 + frame_count)
            if len(pts):
                chunks.append(pts)
                frame_count += 1
    if not chunks:
        return None, f"no frames read from {target.bag_path} topic {target.topic}"
    return np.vstack(chunks), None


def write_frame_metrics(
    target: LoadedTarget,
    transform: np.ndarray,
    groundtruth_tree: cKDTree,
    out_dir: Path,
    frame_cfg: Dict,
    thresholds: Sequence[float],
) -> Dict:
    rosbag, error = import_rosbag()
    if rosbag is None:
        status = {"status": "skipped", "reason": f"rosbag unavailable: {error}"}
        dump_yaml(out_dir / "frame_metrics_status.yaml", status)
        return status
    if target.bag_path is None or not target.bag_path.exists():
        status = {"status": "skipped", "reason": f"bag missing: {target.bag_path}"}
        dump_yaml(out_dir / "frame_metrics_status.yaml", status)
        return status

    max_frames = int(frame_cfg.get("max_frames", 0))
    per_frame_max = int(frame_cfg.get("points_per_frame", 20000))
    rows = []
    start_stamp = None
    with rosbag.Bag(str(target.bag_path), "r") as bag:
        for frame_index, (_, msg, t) in enumerate(bag.read_messages(topics=[target.topic])):
            if max_frames > 0 and frame_index >= max_frames:
                break
            stamp = msg.header.stamp.to_sec() if hasattr(msg, "header") else t.to_sec()
            if start_stamp is None:
                start_stamp = stamp
            pts = sample_points(pointcloud2_to_xyz(msg), per_frame_max, seed=503 + frame_index)
            moved = transform_points(pts, transform)
            distances, _ = groundtruth_tree.query(moved, k=1)
            stats = distance_stats(distances, thresholds)
            row = {
                "frame_index": frame_index,
                "stamp": f"{stamp:.9f}",
                "time_from_start_sec": f"{stamp - start_stamp:.9f}",
                "point_count": stats.get("count", 0),
                "mean": stats.get("mean", math.nan),
                "rmse": stats.get("rmse", math.nan),
                "p50": stats.get("p50", math.nan),
                "p90": stats.get("p90", math.nan),
                "p95": stats.get("p95", math.nan),
                "p99": stats.get("p99", math.nan),
                "max": stats.get("max", math.nan),
            }
            rows.append(row)

    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "frame_metrics.csv"
    fieldnames = ["frame_index", "stamp", "time_from_start_sec", "point_count", "mean", "rmse", "p50", "p90", "p95", "p99", "max"]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    worst_rows = sorted(rows, key=lambda row: float(row["p95"]), reverse=True)[: int(frame_cfg.get("worst_frame_count", 30))]
    with (out_dir / "worst_frames.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(worst_rows)

    p95_values = np.asarray([float(row["p95"]) for row in rows if math.isfinite(float(row["p95"]))], dtype=np.float64)
    summary = {
        "status": "ok",
        "frame_count": int(len(rows)),
        "p95_mean": float(np.mean(p95_values)) if len(p95_values) else None,
        "p95_std": float(np.std(p95_values)) if len(p95_values) else None,
        "p95_p95": float(np.percentile(p95_values, 95)) if len(p95_values) else None,
        "p95_max": float(np.max(p95_values)) if len(p95_values) else None,
        "csv": str(csv_path),
        "worst_frames_csv": str(out_dir / "worst_frames.csv"),
    }
    dump_yaml(out_dir / "frame_metrics_summary.yaml", summary)
    return summary


def default_cases(project_root: Path) -> List[Dict]:
    return [
        {"name": "current_existing", "root": str(project_root / "output/drone_2_gld_indexed")},
        {
            "name": "pgo_global_safe",
            "root": str(project_root / "output/param_sweep/drone_2_gld_indexed/pgo_global_safe"),
        },
        {
            "name": "pgo_global_safe_hba3",
            "root": str(project_root / "output/param_sweep/drone_2_gld_indexed/pgo_global_safe_hba3"),
        },
    ]


def icp_output_dir(icp_root: Path, target: LoadedTarget, mode: str) -> Path:
    return icp_root / target.case_name / mode / target.target_name


def load_targets(case: Dict, project_root: Path, selected_targets: Optional[Sequence[str]]) -> List[LoadedTarget]:
    case_root = resolve_path(case["root"], project_root)
    target_cfg = DEFAULT_TARGETS.copy()
    target_cfg.update(case.get("targets", {}) or {})
    targets = []
    for target_name, cfg in target_cfg.items():
        if selected_targets and target_name not in selected_targets:
            continue
        map_path = resolve_path(cfg["map"], case_root)
        bag_path = resolve_path(cfg["bag"], case_root) if cfg.get("bag") else None
        targets.append(
            LoadedTarget(
                case_name=str(case["name"]),
                target_name=target_name,
                root=case_root,
                map_path=map_path,
                bag_path=bag_path,
                topic=str(cfg.get("topic", "")),
            )
        )
    return targets


def evaluate_target(
    target: LoadedTarget,
    groundtruth: np.ndarray,
    groundtruth_tree: cKDTree,
    output_root: Path,
    icp_root: Path,
    cfg: Dict,
    modes: Sequence[str],
    map_only: bool,
    save_aligned_map: bool,
) -> List[Dict]:
    results = []
    thresholds = [float(v) for v in cfg.get("thresholds_m", [0.05, 0.10, 0.20, 0.50])]
    map_cfg = cfg.get("map", {})
    align_cfg = cfg.get("alignment", {})
    frame_cfg = cfg.get("frame", {})

    if not target.map_path.exists():
        return [
            {
                "case": target.case_name,
                "target": target.target_name,
                "status": "skipped",
                "reason": f"map missing: {target.map_path}",
            }
        ]

    print(f"[LOAD] {target.case_name}/{target.target_name}: {target.map_path}")
    source_map = read_pcd_points(
        target.map_path,
        max_points=int(map_cfg.get("load_max_points", 1200000)),
        voxel_size=float(map_cfg.get("load_voxel_size", 0.0)),
        seed=31,
    )

    transforms: Dict[str, Tuple[Optional[np.ndarray], Dict]] = {}
    if "overall_icp" in modes:
        print(f"[ALIGN] {target.case_name}/{target.target_name}: overall_icp")
        transform, info = align_to_groundtruth(source_map, groundtruth, align_cfg)
        transforms["overall_icp"] = (transform, info)

    if "start_segment_icp" in modes:
        print(f"[ALIGN] {target.case_name}/{target.target_name}: start_segment_icp")
        start_points, reason = load_start_segment_points(target, frame_cfg)
        if start_points is None:
            transforms["start_segment_icp"] = (None, {"status": "skipped", "reason": reason})
        else:
            transform, info = align_to_groundtruth(start_points, groundtruth, align_cfg)
            info["start_segment_points"] = int(len(start_points))
            transforms["start_segment_icp"] = (transform, info)

    for mode, (transform, align_info) in transforms.items():
        out_dir = output_root / target.case_name / mode / target.target_name
        icp_dir = icp_output_dir(icp_root, target, mode)
        out_dir.mkdir(parents=True, exist_ok=True)
        icp_dir.mkdir(parents=True, exist_ok=True)
        if transform is None:
            result = {
                "case": target.case_name,
                "target": target.target_name,
                "mode": mode,
                "status": "skipped",
                "alignment": align_info,
            }
            dump_yaml(out_dir / "map_metrics.yaml", result)
            dump_yaml(icp_dir / "map_metrics.yaml", result)
            results.append(result)
            continue

        moved_map = transform_points(source_map, transform)
        metrics = map_metrics(moved_map, groundtruth, map_cfg, thresholds)
        if save_aligned_map:
            aligned = voxel_downsample(
                moved_map,
                float(map_cfg.get("aligned_map_voxel_size", 0.10)),
                int(map_cfg.get("aligned_map_max_points", 1000000)),
                seed=37,
            )
            write_pcd_xyz(icp_dir / "aligned_map.pcd", aligned)
        transform_doc = {
            "case": target.case_name,
            "target": target.target_name,
            "mode": mode,
            "note": "Rigid lidar/map-to-LAS transform. LAS is evaluation-only and is not used by optimization.",
            "transform_lidar_to_las": transform_to_yaml(transform),
            "alignment": align_info,
        }
        dump_yaml(out_dir / "transform_lidar_to_las.yaml", transform_doc)
        dump_yaml(icp_dir / "transform_lidar_to_las.yaml", transform_doc)
        map_doc = {
            "case": target.case_name,
            "target": target.target_name,
            "mode": mode,
            "status": "ok",
            "map_path": str(target.map_path),
            "bag_path": str(target.bag_path) if target.bag_path else None,
            "topic": target.topic,
            "icp_output_dir": str(icp_dir),
            "aligned_map": str(icp_dir / "aligned_map.pcd") if save_aligned_map else None,
            "transform_lidar_to_las": str(icp_dir / "transform_lidar_to_las.yaml"),
            "metrics": metrics,
        }
        dump_yaml(out_dir / "map_metrics.yaml", map_doc)
        dump_yaml(icp_dir / "map_metrics.yaml", map_doc)

        frame_summary = {"status": "skipped", "reason": "map_only"}
        if not map_only:
            frame_summary = write_frame_metrics(target, transform, groundtruth_tree, out_dir, frame_cfg, thresholds)
        result = {
            "case": target.case_name,
            "target": target.target_name,
            "mode": mode,
            "status": "ok",
            "map_metrics": metrics,
            "frame_metrics": frame_summary,
            "output_dir": str(out_dir),
            "icp_output_dir": str(icp_dir),
            "aligned_map": str(icp_dir / "aligned_map.pcd") if save_aligned_map else None,
            "transform_lidar_to_las": str(icp_dir / "transform_lidar_to_las.yaml"),
        }
        results.append(result)
    return results


def summarize_results(results: List[Dict], output_root: Path) -> Dict:
    ok_results = [r for r in results if r.get("status") == "ok"]

    def map_p95(result: Dict) -> float:
        return float(result["map_metrics"]["source_to_groundtruth"]["p95"])

    def frame_key(result: Dict) -> Tuple[float, float, float]:
        frame = result.get("frame_metrics", {})
        if frame.get("status") != "ok":
            return (float("inf"), float("inf"), float("inf"))
        return (
            float(frame.get("p95_mean", float("inf"))),
            float(frame.get("p95_std", float("inf"))),
            float(frame.get("p95_p95", float("inf"))),
        )

    overall_rank = sorted(
        [r for r in ok_results if r.get("mode") == "overall_icp"],
        key=map_p95,
    )
    start_frame_rank = sorted(
        [r for r in ok_results if r.get("mode") == "start_segment_icp"],
        key=frame_key,
    )

    summary = {
        "note": "LAS真值只用于离线评价，不参与PGO/HBA优化。",
        "overall_icp_map_p95_rank": [
            {
                "case": r["case"],
                "target": r["target"],
                "source_to_groundtruth_p95": map_p95(r),
                "output_dir": r["output_dir"],
                "icp_output_dir": r["icp_output_dir"],
                "aligned_map": r.get("aligned_map"),
                "transform_lidar_to_las": r.get("transform_lidar_to_las"),
            }
            for r in overall_rank
        ],
        "start_segment_icp_frame_p95_rank": [
            {
                "case": r["case"],
                "target": r["target"],
                "frame_p95_mean": r.get("frame_metrics", {}).get("p95_mean"),
                "frame_p95_std": r.get("frame_metrics", {}).get("p95_std"),
                "frame_p95_p95": r.get("frame_metrics", {}).get("p95_p95"),
                "frame_p95_max": r.get("frame_metrics", {}).get("p95_max"),
                "output_dir": r["output_dir"],
                "icp_output_dir": r["icp_output_dir"],
                "aligned_map": r.get("aligned_map"),
                "transform_lidar_to_las": r.get("transform_lidar_to_las"),
            }
            for r in start_frame_rank
        ],
        "skipped": [
            {
                "case": r.get("case"),
                "target": r.get("target"),
                "mode": r.get("mode"),
                "reason": r.get("reason") or r.get("alignment", {}).get("reason"),
            }
            for r in results
            if r.get("status") == "skipped"
        ],
        "results": results,
    }
    dump_yaml(output_root / "summary_compare.yaml", summary)
    write_markdown_report(output_root / "report.md", summary)
    return summary


def write_markdown_report(path: Path, summary: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# LAS Ground Truth Evaluation Report\n\n")
        f.write("LAS真值只用于离线评价，不参与PGO/HBA优化。\n\n")
        f.write("## Overall ICP Map Ranking\n\n")
        f.write("| Rank | Case | Target | Source->GT p95 (m) | ICP Output |\n")
        f.write("|---:|---|---|---:|---|\n")
        for idx, row in enumerate(summary.get("overall_icp_map_p95_rank", []), 1):
            f.write(
                f"| {idx} | {row['case']} | {row['target']} | "
                f"{row['source_to_groundtruth_p95']:.6f} | `{row['icp_output_dir']}` |\n"
            )
        f.write("\n## Start Segment ICP Frame-p95 Ranking\n\n")
        f.write("| Rank | Case | Target | Frame p95 mean (m) | Frame p95 std (m) | Frame p95 p95 (m) | Max (m) |\n")
        f.write("|---:|---|---|---:|---:|---:|---:|\n")
        for idx, row in enumerate(summary.get("start_segment_icp_frame_p95_rank", []), 1):
            if row["frame_p95_mean"] is None:
                continue
            f.write(
                f"| {idx} | {row['case']} | {row['target']} | "
                f"{float(row['frame_p95_mean']):.6f} | {float(row['frame_p95_std']):.6f} | "
                f"{float(row['frame_p95_p95']):.6f} | {float(row['frame_p95_max']):.6f} |\n"
            )
        f.write("\n## Notes\n\n")
        f.write("- `overall_icp` 评价整图最终形状贴合真值的能力。\n")
        f.write("- `start_segment_icp` 只用bag起始段对齐，再评价全程漂移、分层和闭合一致性。\n")
        f.write("- `frame_metrics.csv` 是逐帧点云到LAS真值的最近邻误差，主看每帧 `p95` 的均值、标准差和最差帧。\n")
        skipped = summary.get("skipped", [])
        if skipped:
            f.write("\n## Skipped Items\n\n")
            f.write("| Case | Target | Mode | Reason |\n")
            f.write("|---|---|---|---|\n")
            for row in skipped:
                f.write(
                    f"| {row.get('case')} | {row.get('target')} | {row.get('mode')} | "
                    f"{row.get('reason')} |\n"
                )


def apply_cli_overrides(cfg: Dict, args: argparse.Namespace) -> Dict:
    if args.output_root:
        cfg["output_root"] = args.output_root
    if args.groundtruth_max_points:
        cfg.setdefault("groundtruth", {})["load_max_points"] = args.groundtruth_max_points
    if args.map_max_points:
        cfg.setdefault("map", {})["load_max_points"] = args.map_max_points
    if args.alignment_max_points:
        cfg.setdefault("alignment", {})["source_max_points"] = args.alignment_max_points
    if args.metric_max_points:
        cfg.setdefault("map", {})["metric_source_max_points"] = args.metric_max_points
        cfg.setdefault("map", {})["metric_groundtruth_max_points"] = args.metric_max_points
    if args.frame_max_frames is not None:
        cfg.setdefault("frame", {})["max_frames"] = args.frame_max_frames
    return cfg


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate PGO/HBA outputs against LAS ground truth.")
    parser.add_argument("--config", default="tools/groundtruth_eval/groundtruth_eval.yaml")
    parser.add_argument("--project-root", default=None)
    parser.add_argument("--output-root", default=None)
    parser.add_argument("--icp-output-root", default=None)
    parser.add_argument("--cases", default=None, help="Comma-separated case names.")
    parser.add_argument("--targets", default=None, help="Comma-separated target names: input,pgo,hba.")
    parser.add_argument("--modes", default=None, help="Comma-separated modes: overall_icp,start_segment_icp.")
    parser.add_argument("--map-only", action="store_true", help="Skip per-frame bag metrics.")
    parser.add_argument("--no-save-aligned-map", action="store_true")
    parser.add_argument("--groundtruth-max-points", type=int, default=None)
    parser.add_argument("--map-max-points", type=int, default=None)
    parser.add_argument("--alignment-max-points", type=int, default=None)
    parser.add_argument("--metric-max-points", type=int, default=None)
    parser.add_argument("--frame-max-frames", type=int, default=None)
    args = parser.parse_args()

    project_root = resolve_path(args.project_root, Path.cwd()) if args.project_root else project_root_from_script()
    config_path = resolve_path(args.config, project_root)
    cfg = apply_cli_overrides(load_yaml(config_path), args)
    output_root = resolve_path(cfg.get("output_root", "output/groundtruth_eval"), project_root)
    icp_output_root = resolve_path(
        args.icp_output_root or cfg.get("icp_output_root", "output/ICP"),
        project_root,
    )
    output_root.mkdir(parents=True, exist_ok=True)
    icp_output_root.mkdir(parents=True, exist_ok=True)

    selected_cases = set(args.cases.split(",")) if args.cases else None
    selected_targets = args.targets.split(",") if args.targets else None
    modes = args.modes.split(",") if args.modes else cfg.get("modes", ["overall_icp", "start_segment_icp"])

    gt_cfg = cfg.get("groundtruth", {})
    gt_path = resolve_path(gt_cfg.get("path", "gld_fr_groundtruth.las"), project_root)
    print(f"[LOAD] groundtruth: {gt_path}")
    groundtruth = read_las_points(
        gt_path,
        max_points=int(gt_cfg.get("load_max_points", 1500000)),
        voxel_size=float(gt_cfg.get("load_voxel_size", 0.0)),
        seed=3,
    )
    groundtruth_tree_points = voxel_downsample(
        groundtruth,
        float(gt_cfg.get("frame_tree_voxel_size", 0.10)),
        int(gt_cfg.get("frame_tree_max_points", 800000)),
        seed=5,
    )
    groundtruth_tree = cKDTree(groundtruth_tree_points)
    dump_yaml(
        output_root / "groundtruth_info.yaml",
        {
            "path": str(gt_path),
            "loaded_points": int(len(groundtruth)),
            "frame_tree_points": int(len(groundtruth_tree_points)),
            "bounds_min": [float(v) for v in groundtruth.min(axis=0)],
            "bounds_max": [float(v) for v in groundtruth.max(axis=0)],
        },
    )

    case_cfgs = cfg.get("cases") or default_cases(project_root)
    if selected_cases:
        case_cfgs = [case for case in case_cfgs if str(case.get("name")) in selected_cases]
    if not case_cfgs:
        raise RuntimeError("No cases selected")

    results: List[Dict] = []
    for case in case_cfgs:
        for target in load_targets(case, project_root, selected_targets):
            results.extend(
                evaluate_target(
                    target,
                    groundtruth,
                    groundtruth_tree,
                    output_root,
                    icp_output_root,
                    cfg,
                    modes=modes,
                    map_only=args.map_only,
                    save_aligned_map=not args.no_save_aligned_map,
                )
            )

    summarize_results(results, output_root)
    print(f"[DONE] report: {output_root / 'report.md'}")
    print(f"[DONE] summary: {output_root / 'summary_compare.yaml'}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise
