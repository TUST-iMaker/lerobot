"""SO-ARM101 示教数据导出为 LeRobot 官方数据集格式转换器.

将 recording/data/<task>/episode_XXXXXX.jsonl 示教轨迹及关联的相机画面，
格式化转换为符合 Hugging Face LeRobot (v2.0) 规范的标准化数据集结构：
1. 提取 observations.state (6轴关节角度) 与 action (6轴动作目标);
2. 提取末端 TCP 笛卡尔坐标与时间戳序列;
3. 生成 meta/info.json (包含特征维度定义、采样频率、任务名称与机器人型号) 与 meta/episodes.jsonl;
4. 汇总序列化为标准化 Parquet / JSON 格式，可直接供 lerobot-train 训练 ACT / Diffusion Policy.
"""

from __future__ import annotations

import json
import logging
import shutil
import time
from pathlib import Path
from typing import Any

from core.data_types import SO101_JOINT_NAMES, rad_to_deg
from recording.cleaner import EpisodeData

logger = logging.getLogger(__name__)


class LeRobotExporter:
    """LeRobot 标准数据集导出器."""

    def __init__(self, output_root: Path | str | None = None) -> None:
        if output_root is None:
            self.output_root = Path(__file__).resolve().parent / "data" / "lerobot_export"
        else:
            self.output_root = Path(output_root).resolve()

    def export_episode(
        self,
        episode: EpisodeData,
        repo_id: str = "so101_custom_dataset",
        fps: int = 30,
        copy_images: bool = True,
    ) -> Path:
        """将单个 EpisodeData 导出为 LeRobot 规范的数据集目录."""
        dataset_dir = self.output_root / repo_id
        meta_dir = dataset_dir / "meta"
        data_dir = dataset_dir / "data"
        videos_dir = dataset_dir / "videos"

        meta_dir.mkdir(parents=True, exist_ok=True)
        data_dir.mkdir(parents=True, exist_ok=True)
        videos_dir.mkdir(parents=True, exist_ok=True)

        frames_data = episode.frames
        total_frames = len(frames_data)
        if total_frames == 0:
            raise ValueError("待导出的示教轨迹为空")

        logger.info("开始导出轨迹: %s (共 %d 帧) -> %s", episode.task_name, total_frames, dataset_dir)

        # 1. 整理各帧特征矩阵
        standardized_rows: list[dict[str, Any]] = []
        for idx, f in enumerate(frames_data):
            joint_pos = f.get("joint_positions", {})
            action_pos = f.get("target_joint_positions", {}) or joint_pos

            state_vec = [float(joint_pos.get(j, 0.0)) for j in SO101_JOINT_NAMES]
            action_vec = [float(action_pos.get(j, 0.0)) for j in SO101_JOINT_NAMES]

            tcp = f.get("tcp_pose", {})
            row = {
                "episode_index": episode.episode_id,
                "frame_index": idx,
                "timestamp": round(idx / float(fps), 4),
                "observation.state": state_vec,
                "action": action_vec,
                "observation.tcp": [
                    tcp.get("x", 0.0),
                    tcp.get("y", 0.0),
                    tcp.get("z", 0.0),
                    tcp.get("roll", 0.0),
                    tcp.get("pitch", 0.0),
                    tcp.get("yaw", 0.0),
                ],
                "control_source": f.get("control_source", "unknown"),
            }
            standardized_rows.append(row)

        # 2. 导出序列数据 (data/episode_XXXXXX.jsonl 或 parquet)
        ep_file = data_dir / f"episode_{episode.episode_id:06d}.jsonl"
        with open(ep_file, "w", encoding="utf-8") as fp:
            for row in standardized_rows:
                fp.write(json.dumps(row) + "\n")

        # 3. 复制相关相机帧 (如果存在)
        if copy_images and episode.filepath:
            raw_dir = episode.filepath.parent
            # 检查同级目录下是否有 images/ 或关联截图
            images_src = raw_dir / "images"
            if images_src.is_dir():
                target_img_dir = dataset_dir / "images" / f"episode_{episode.episode_id:06d}"
                target_img_dir.mkdir(parents=True, exist_ok=True)
                for img_f in images_src.glob("*.jpg"):
                    shutil.copy2(img_f, target_img_dir / img_f.name)

        # 4. 生成 meta/info.json
        info_json = {
            "codebase_version": "v2.0",
            "robot_type": "so_arm101",
            "total_episodes": 1,
            "total_frames": total_frames,
            "total_tasks": 1,
            "fps": fps,
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [len(SO101_JOINT_NAMES)],
                    "names": list(SO101_JOINT_NAMES),
                },
                "action": {
                    "dtype": "float32",
                    "shape": [len(SO101_JOINT_NAMES)],
                    "names": list(SO101_JOINT_NAMES),
                },
                "observation.tcp": {
                    "dtype": "float32",
                    "shape": [6],
                    "names": ["x", "y", "z", "roll", "pitch", "yaw"],
                },
                "timestamp": {"dtype": "float32", "shape": [1]},
                "frame_index": {"dtype": "int64", "shape": [1]},
                "episode_index": {"dtype": "int64", "shape": [1]},
            },
            "task": episode.task_name,
        }

        with open(meta_dir / "info.json", "w", encoding="utf-8") as fp:
            json.dump(info_json, fp, indent=2, ensure_ascii=False)

        # 5. 生成 meta/episodes.jsonl
        meta_ep_file = meta_dir / "episodes.jsonl"
        meta_row = {
            "episode_index": episode.episode_id,
            "tasks": [episode.task_name],
            "length": total_frames,
        }
        with open(meta_ep_file, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(meta_row) + "\n")

        logger.info("[✓] LeRobot 数据集已成功导出至: %s", dataset_dir)
        return dataset_dir
