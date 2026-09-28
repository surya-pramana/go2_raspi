"""Isolate USB acquisition, alignment and colorization without network or GPIO.

Run on the Pi with the same Python and privileges as the server. Output is a
timestamped text log including SDK diagnostics and stack dumps for stalled calls.
"""
import argparse
import faulthandler
import os
import platform
import sys
import time
import traceback
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['acquire', 'align', 'colorize'], default='acquire')
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--idle-seconds', type=float, default=0,
                        help='Wait after startup, like a server waiting for its client')
    parser.add_argument('--output-dir', default='debug_logs')
    args = parser.parse_args()
    if args.frames < 1 or args.cycles < 1 or args.idle_seconds < 0:
        parser.error('frames/cycles must be positive and idle-seconds nonnegative')
    directory = Path(args.output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    log = directory / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_{args.stage}.log')
    print(f'Diagnostic log: {log.resolve()}', flush=True)
    # Redirect file descriptors too, capturing native SDK diagnostics.
    with log.open('w', encoding='utf-8') as output:
        saved = os.dup(1), os.dup(2)
        try:
            sys.stdout.flush()
            sys.stderr.flush()
            os.dup2(output.fileno(), 1)
            os.dup2(output.fileno(), 2)
            faulthandler.enable()
            faulthandler.dump_traceback_later(30, repeat=True)
            print('Python:', sys.executable, sys.version, flush=True)
            print('Platform:', platform.platform(), flush=True)
            print('PYTHONPATH:', os.environ.get('PYTHONPATH'), flush=True)
            print('Arguments:', vars(args), flush=True)
            import pyrealsense2 as rs
            print('SDK module:', rs.__file__, flush=True)
            try:
                print('SDK distribution:', version('pyrealsense2'), flush=True)
            except PackageNotFoundError:
                print('SDK distribution version unknown', flush=True)
            rs.log_to_console(rs.log_severity.debug)
            for cycle in range(1, args.cycles + 1):
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                started = False
                operation = 'pipeline.start'
                try:
                    print(f'CYCLE {cycle}: {operation}', flush=True)
                    pipeline.start(config)
                    started = True
                    if args.idle_seconds:
                        print(f'Idle for {args.idle_seconds}s', flush=True)
                        time.sleep(args.idle_seconds)
                    align = rs.align(rs.stream.color) if args.stage != 'acquire' else None
                    colorizer = rs.colorizer() if args.stage == 'colorize' else None
                    for index in range(args.frames):
                        operation = f'frame {index + 1}: wait_for_frames'
                        print(operation, flush=True)
                        frames = pipeline.wait_for_frames(5000)
                        color, depth = frames.get_color_frame(), frames.get_depth_frame()
                        if not color or not depth:
                            raise RuntimeError('Incomplete RGB/depth frameset')
                        print(f'IDs RGB={color.get_frame_number()} depth={depth.get_frame_number()}',
                              flush=True)
                        if align is not None:
                            operation = f'frame {index + 1}: align.process'
                            print(operation, flush=True)
                            frames = align.process(frames)
                            if not frames.get_color_frame() or not frames.get_depth_frame():
                                raise RuntimeError('Incomplete aligned frameset')
                        if colorizer is not None:
                            operation = f'frame {index + 1}: colorizer.colorize'
                            print(operation, flush=True)
                            colorizer.colorize(frames.get_depth_frame())
                        # Release SDK frame references before stopping/reopening.
                        del color, depth, frames
                    print(f'CYCLE {cycle}: PASS', flush=True)
                except BaseException:
                    print(f'CYCLE {cycle}: FAILED AT {operation}', flush=True)
                    raise
                finally:
                    if started:
                        print(f'CYCLE {cycle}: pipeline.stop begin', flush=True)
                        pipeline.stop()
                        print(f'CYCLE {cycle}: pipeline.stop complete', flush=True)
                del pipeline
                if cycle < args.cycles:
                    time.sleep(2)
            print('ALL CYCLES PASSED', flush=True)
            result = 0
        except BaseException:
            traceback.print_exc()
            result = 1
        finally:
            faulthandler.cancel_dump_traceback_later()
            faulthandler.disable()
            sys.stdout.flush()
            sys.stderr.flush()
            os.dup2(saved[0], 1)
            os.dup2(saved[1], 2)
            os.close(saved[0])
            os.close(saved[1])
    print(f'Completed: {"PASS" if result == 0 else "FAIL"}. Log: {log.resolve()}', flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
