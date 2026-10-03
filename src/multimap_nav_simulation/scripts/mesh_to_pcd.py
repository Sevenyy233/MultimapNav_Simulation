#!/usr/bin/env python3
"""使用 Open3D 将 PLY/STL 三角网格转换为 PCD 点云。

命令行教程
==========

1. 转换单个 STL 文件（均匀采样 10 万点）：

   python3 mesh_to_pcd.py ../../xxx.stl -o ../../xxx.pcd \
       --points 100000 --method uniform

2. 转换单个 PLY 文件（泊松盘采样 20 万点）：

   python3 mesh_to_pcd.py ../../xxx.ply -o ../../xxx.pcd \
       --points 200000 --method poisson

3. 批量转换默认 ``../3DModel`` 目录中的 PLY/STL 文件：

   python3 mesh_to_pcd.py

4. 批量转换指定目录，并输出到另一个目录：

   python3 mesh_to_pcd.py ../../3DModels -o ../../pcd_output \
       --points 100000 --method uniform

参数说明
========

* 第一个位置参数：输入的 ``.stl``/``.ply`` 文件或包含模型的目录。
* ``-o, --output``：输出的 ``.pcd`` 文件或输出目录。
* ``-n, --points``：每个网格表面采样的点数，必须大于 0。
* ``--method uniform``：均匀随机采样，速度较快，是默认方式。
* ``--method poisson``：泊松盘采样，点分布通常更均匀，但速度较慢。
* ``--ascii``：输出 ASCII PCD；默认输出体积更小的二进制 PCD。
* ``--normals``：在 PCD 中额外保存表面法向量。
* ``--overwrite``：允许覆盖已经存在的输出文件。

运行时只使用终端中的同一行显示动态进度，不会为每次刷新打印一条新日志。
进度行会显示动态指示器、当前文件、处理阶段和该阶段已用时间；全部结束后
才保留最终汇总。Open3D 的单个网格采样接口不提供内部百分比回调，因此
采样期间使用往复移动的进度指示器，不显示虚假的采样百分比；批量转换的
已完成比例按实际处理完成的文件数计算。

采样点数建议
============

* 1 万～3 万点：快速预览、检查坐标和模型方向。
* 5 万～10 万点：普通小型或中型模型，适合可视化和初步测试。
* 10 万～30 万点：地形、导航和建图使用，通常能兼顾细节与处理速度。
* 50 万点以上：保留较小的表面细节，但文件、内存和处理时间会明显增加。

点数应根据模型表面积和目标点间距选择，而不只是根据文件大小选择。可用
``点数约等于模型表面积除以目标点间距的平方`` 进行估算。例如表面积约为
1000 平方米，希望平均点间距约为 0.1 米，可先尝试 10 万点。当前 terrain
模型默认使用 10 万点比较合适；如果导航地图分辨率更高或地面细节不足，
可增加到 20 万～30 万点。建议先用 10 万点转换，再根据效果逐步增加。

相对路径以执行命令时所在的目录为基准。单文件输入配合 ``.pcd`` 输出路径
时会生成指定文件；目录输入时会递归转换，并自动使用 ``_ply.pcd`` 或
``_stl.pcd`` 后缀，避免同名 PLY/STL 互相覆盖。
"""

from __future__ import annotations

import argparse
import multiprocessing
import sys
import threading
import time
from pathlib import Path


SUPPORTED_SUFFIXES = {".ply", ".stl"}
PACKAGE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = PACKAGE_DIR / "3DModel"
DEFAULT_OUTPUT = PACKAGE_DIR / "pcd"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "将 PLY/STL 三角网格表面采样为 PCD 点云；输入目录时会递归查找。"
        ),
        epilog=(
            "示例：\n"
            "  python3 mesh_to_pcd.py ../../xxx.stl -o ../../xxx.pcd "
            "-n 100000 --method uniform\n"
            "  python3 mesh_to_pcd.py ../../xxx.ply -o ../../xxx.pcd "
            "-n 200000 --method poisson --overwrite"
        ),
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"PLY/STL 文件或模型目录（默认：{DEFAULT_INPUT}）",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"输出 PCD 文件或目录（默认：{DEFAULT_OUTPUT}）",
    )
    parser.add_argument(
        "-n",
        "--points",
        type=int,
        default=100_000,
        help="每个网格的表面采样点数（默认：100000）",
    )
    parser.add_argument(
        "--method",
        choices=("uniform", "poisson"),
        default="uniform",
        help="采样方式：uniform 较快，poisson 更均匀（默认：uniform）",
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="输出可读的 ASCII PCD，而不是默认的较小二进制格式",
    )
    parser.add_argument(
        "--normals",
        action="store_true",
        help="在 PCD 中保存表面法向量",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="覆盖已经存在的输出文件",
    )
    return parser.parse_args()


def find_meshes(input_path: Path) -> list[Path]:
    if input_path.is_file():
        if input_path.suffix.lower() not in SUPPORTED_SUFFIXES:
            raise ValueError(f"unsupported input format: {input_path.suffix}")
        return [input_path]

    if not input_path.is_dir():
        raise FileNotFoundError(f"input does not exist: {input_path}")

    return sorted(
        path
        for path in input_path.rglob("*")
        if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
    )


def output_path_for(
    mesh_path: Path, input_path: Path, output_path: Path, batch: bool
) -> Path:
    if not batch and output_path.suffix.lower() == ".pcd":
        return output_path

    relative_parent = (
        mesh_path.relative_to(input_path).parent if input_path.is_dir() else Path()
    )
    # Include the source extension so same-named terrain.ply and terrain.stl do
    # not overwrite one another.
    filename = f"{mesh_path.stem}_{mesh_path.suffix[1:].lower()}.pcd"
    return output_path / relative_parent / filename


def xyz_only(open3d, point_cloud):
    result = open3d.geometry.PointCloud()
    result.points = point_cloud.points
    return result


def convert_mesh(
    open3d,
    mesh_path: Path,
    point_count: int,
    method: str,
    normals: bool,
    report_progress,
):
    report_progress(f"读取三角网格：{mesh_path}")
    mesh = open3d.io.read_triangle_mesh(str(mesh_path))
    if mesh.is_empty() or not mesh.has_triangles():
        raise ValueError("the file does not contain a non-empty triangle mesh")

    if normals:
        report_progress("计算表面法向量")
        mesh.compute_vertex_normals()

    method_name = "泊松盘" if method == "poisson" else "均匀"
    report_progress(f"{method_name}采样 {point_count} 个点")
    if method == "poisson":
        cloud = mesh.sample_points_poisson_disk(number_of_points=point_count)
    else:
        cloud = mesh.sample_points_uniformly(number_of_points=point_count)

    if cloud.is_empty():
        raise RuntimeError("Open3D produced an empty point cloud")
    return cloud if normals else xyz_only(open3d, cloud)


def conversion_worker(
    mesh_path_text: str,
    destination_text: str,
    point_count: int,
    method: str,
    normals: bool,
    write_ascii: bool,
    connection,
) -> None:
    """Run Open3D in another process so the parent can keep refreshing progress."""
    try:
        try:
            import open3d
        except ImportError as error:
            raise RuntimeError(
                "Open3D is not installed. Install the compatible smaller release "
                "with \"python3 -m pip install --user 'numpy<1.25' "
                "open3d==0.17.0\"."
            ) from error

        mesh_path = Path(mesh_path_text)
        destination = Path(destination_text)

        def report_progress(message: str) -> None:
            connection.send(("stage", message))

        cloud = convert_mesh(
            open3d, mesh_path, point_count, method, normals, report_progress
        )
        report_progress("写入 PCD")
        destination.parent.mkdir(parents=True, exist_ok=True)
        written = open3d.io.write_point_cloud(
            str(destination), cloud, write_ascii=write_ascii, compressed=False
        )
        if not written:
            raise RuntimeError("Open3D failed to write the PCD file")
        connection.send(("success", len(cloud.points)))
    except Exception as error:
        connection.send(("error", str(error)))
    finally:
        connection.close()


def convert_in_subprocess(
    mesh_path: Path,
    destination: Path,
    point_count: int,
    method: str,
    normals: bool,
    write_ascii: bool,
    report_progress,
) -> int:
    context = multiprocessing.get_context("spawn")
    parent_connection, child_connection = context.Pipe(duplex=False)
    process = context.Process(
        target=conversion_worker,
        args=(
            str(mesh_path),
            str(destination),
            point_count,
            method,
            normals,
            write_ascii,
            child_connection,
        ),
    )
    process.start()
    child_connection.close()
    result = None
    error_message = None

    def handle_message(message) -> None:
        nonlocal result, error_message
        kind, value = message
        if kind == "stage":
            report_progress(value)
        elif kind == "success":
            result = value
        elif kind == "error":
            error_message = value

    try:
        while process.is_alive():
            if parent_connection.poll(0.1):
                try:
                    handle_message(parent_connection.recv())
                except EOFError:
                    break
        process.join()
        while parent_connection.poll():
            try:
                handle_message(parent_connection.recv())
            except EOFError:
                break
    except BaseException:
        if process.is_alive():
            process.terminate()
        process.join()
        raise
    finally:
        parent_connection.close()

    if error_message is not None:
        raise RuntimeError(error_message)
    if process.exitcode != 0:
        raise RuntimeError(f"conversion process exited with code {process.exitcode}")
    if result is None:
        raise RuntimeError("conversion process returned no result")
    return result


class ProgressDisplay:
    """Continuously refresh one terminal line without fabricating percentages."""

    BAR_WIDTH = 24

    def __init__(self, total: int) -> None:
        self.total = total
        self.completed = 0
        self.current = 0
        self.filename = ""
        self.stage = "准备"
        self.stage_started = time.monotonic()
        self.interactive = sys.stdout.isatty()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self) -> None:
        if not self.interactive:
            return
        self._thread = threading.Thread(target=self._animate, daemon=True)
        self._thread.start()

    def update(self, current: int, filename: str, stage: str) -> None:
        with self._lock:
            self.current = current
            self.filename = filename
            self.stage = stage
            self.stage_started = time.monotonic()

    def complete(self, current: int, filename: str, result: str) -> None:
        with self._lock:
            self.completed = current
            self.current = current
            self.filename = filename
            self.stage = result
            self.stage_started = time.monotonic()

    def finish(self, summary: str, mark_all_processed: bool = True) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()

        completed = self.total if mark_all_processed else self.completed
        ratio = completed / self.total
        filled = round(self.BAR_WIDTH * ratio)
        bar = "#" * filled + "-" * (self.BAR_WIDTH - filled)
        if self.interactive:
            sys.stdout.write(
                f"\r\033[2K总体进度 [{bar}] {completed}/{self.total} "
                f"({ratio * 100:.1f}%) | {summary}\n"
            )
            sys.stdout.flush()
        else:
            print(
                f"总体进度 {completed}/{self.total} "
                f"({ratio * 100:.1f}%) | {summary}"
            )

    def _animate(self) -> None:
        tick = 0
        while not self._stop_event.wait(0.12):
            self._draw(tick)
            tick += 1

    def _draw(self, tick: int) -> None:
        with self._lock:
            completed = self.completed
            current = self.current
            filename = self.filename
            stage = self.stage
            elapsed = time.monotonic() - self.stage_started

        filled = int(self.BAR_WIDTH * completed / self.total)
        cells = ["#"] * filled + ["-"] * (self.BAR_WIDTH - filled)
        if filled < self.BAR_WIDTH:
            remaining = self.BAR_WIDTH - filled
            cycle = max(1, remaining * 2 - 2)
            offset = tick % cycle
            if offset >= remaining:
                offset = cycle - offset
            cells[filled + offset] = ">"

        status = (
            f"总体进度 [{''.join(cells)}] 已处理 {completed}/{self.total}"
        )
        if current:
            status += f" | [{current}/{self.total}] {stage}：{filename} ({elapsed:.1f}s)"

        sys.stdout.write(f"\r\033[2K{status}")
        sys.stdout.flush()


def run_conversion(args: argparse.Namespace) -> int:
    if args.points <= 0:
        raise ValueError("--points must be greater than zero")

    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    meshes = find_meshes(input_path)
    if not meshes:
        print(f"No PLY/STL files found under {input_path}", file=sys.stderr)
        return 1

    batch = input_path.is_dir() or len(meshes) > 1
    failures = 0
    succeeded = 0
    skipped = 0
    errors = []
    total = len(meshes)
    progress = ProgressDisplay(total)
    progress.start()

    try:
        for index, mesh_path in enumerate(meshes, start=1):

            def report_progress(message: str) -> None:
                progress.update(index, mesh_path.name, message)

            destination = output_path_for(mesh_path, input_path, output_path, batch)
            if destination.exists() and not args.overwrite:
                skipped += 1
                progress.complete(index, mesh_path.name, "跳过（输出已存在）")
                continue

            try:
                report_progress("启动 Open3D 转换进程")
                point_count = convert_in_subprocess(
                    mesh_path,
                    destination,
                    args.points,
                    args.method,
                    args.normals,
                    args.ascii,
                    report_progress,
                )
                succeeded += 1
                progress.complete(
                    index, mesh_path.name, f"完成（{point_count} 个点）"
                )
            except Exception as error:  # Continue converting the remaining batch.
                failures += 1
                errors.append(f"{mesh_path}: {error}")
                progress.complete(index, mesh_path.name, "失败")
    except KeyboardInterrupt:
        progress.finish(
            f"已中止；成功 {succeeded}，跳过 {skipped}，失败 {failures}",
            mark_all_processed=False,
        )
        return 130

    progress.finish(f"成功 {succeeded}，跳过 {skipped}，失败 {failures}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)

    return 1 if failures else 0


def main() -> int:
    try:
        return run_conversion(parse_args())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
