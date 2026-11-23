#!/usr/bin/env python3
"""Runs LiDAR(-inertial) odometry methods on the MVSim LIO benchmark sequences.

One job = (variant, world/traj, method). Each job runs single-threaded, pinned
to one core taken from a core list that is re-read before every launch (so the
share of the machine can be changed while a batch runs), and only while the
memory headroom allows it.

Outputs, under --out (default $LIO_ROOT/runs):
  <variant>/<world>__<traj>/<method>.tum    estimated trajectory (TUM)
  <variant>/<world>__<traj>/<method>.log    console log (progress bars removed)
  <variant>/<world>__<traj>/<method>.time   wall / user / sys time and max RSS

Usage:
  run_lio.py --variants os1_32-real-rolling,os1_32-real-global \
             --methods mola_lo,kiss,glim,dlio,fastlio2 [--seqs world/traj,...]
             [--cores-file cores.txt] [--max-jobs 16] [--mem-cap-gb 20]
"""
import argparse
import contextlib
import fcntl
import json
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CFG = HERE / 'configs'
# Data root (LIO_DATA): the released variants, or another tree with the same layout.
DATA = Path(os.environ.get('LIO_DATA', Path.home() / 'mvsim_lio'))
TOOLS = HERE.parent / 'tools'
# Every job writes below this directory (LIO_ROOT), which the memory accounting scans.
EFFORT_ROOT = Path(os.environ.get('LIO_ROOT', Path.home() / 'mvsim_lio_eval'))

# rig: (min range [m], max range [m], rings, vertical FOV [deg])
RIGS = {'os1_32': (0.5, 60.0, 32, 45.0), 'vlp16': (0.4, 100.0, 16, 30.0),
        'os1_64': (0.5, 60.0, 64, 45.0), 'os0_128': (0.3, 50.0, 128, 90.0),
        'os1_16': (0.5, 120.0, 16, 33.2)}

# Base workspace (LIO_WS): the default one, or a frozen copy of it that keeps the binaries
# of earlier results while the default one is rebuilt.
LIO_WS = os.environ.get('LIO_WS', '$HOME/ros2_ws/install')
# Optional underlay between ROS and LIO_WS (LIO_UNDERLAY), e.g. a separate MOLA build.
ENV_SETUP = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'lyrical')}/setup.bash"
if os.environ.get('LIO_UNDERLAY'):
    ENV_SETUP += f" && source {os.environ['LIO_UNDERLAY']}/setup.bash"
ENV_SETUP += f' && source {LIO_WS}/setup.bash'
# Optional overlay workspace on top, e.g. a newer build of some packages. Its
# local_setup only adds the overlay: its setup.bash would source again the workspace
# it was built on, ahead of LIO_WS.
if os.environ.get('LIO_OVERLAY'):
    ENV_SETUP += f" && source {os.environ['LIO_OVERLAY']}/local_setup.bash"

ONE_THREAD = {'OMP_NUM_THREADS': '1', 'TBB_NUM_THREADS': '1', 'NUM_THREADS': '1',
              'MOLA_MAX_THREADS': '1', 'MP_PROC_NUM': '1',
              'MOLA_ASYNC_BACKEND': 'false', 'MOLA_INCREMENTAL_MAP_ASYNC_REBUILD': 'false'}


def tuning_env(base, tag):
    """Overrides of one combination of the tuning grid: tag 't' + one bit per factor."""
    factors = json.loads((CFG / 'tuning.json').read_text())['methods'][base]['factors']
    bits = tag[1:]
    if len(bits) != len(factors):
        raise ValueError(f'{base}+{tag}: expected {len(factors)} bits')
    env = {}
    for bit, f in zip(bits, factors):
        if bit == '1':
            env.update(f['env'])
    return env


# KISS-ICP takes its tuning factors as command line options.
KISS_OPTIONS = {'KISS_MAX_POINTS_PER_VOXEL': '--max-points-per-voxel',
                'KISS_INITIAL_THRESHOLD': '--initial-threshold',
                'KISS_MIN_MOTION_TH': '--min-motion-th'}


def kiss_args(env, max_range):
    args = []
    if 'KISS_VOXEL_PER_RANGE' in env:
        args += ['--voxel-size', f'{float(env["KISS_VOXEL_PER_RANGE"]) * max_range:g}']
    for k, opt in KISS_OPTIONS.items():
        if k in env:
            args += [opt, env[k]]
    return args


def method_cmd(method, rawlog, out_tum, rig, shutter):
    """Command line and environment of one method on one rawlog.

    "<method>+t<bits>" runs <method> with one combination of the tuning grid
    (configs/tuning.json); "<method>+<tag>" with the extra environment
    variables of configs/overrides.json[<tag>]. Either is one configuration
    for every sequence."""
    if '+' in method:
        base, tag = method.split('+', 1)
        cmd, env = method_cmd(base, rawlog, out_tum, rig, shutter)
        if re.fullmatch(r't[01]+', tag):
            extra = tuning_env(base, tag)
        else:
            extra = json.loads((CFG / 'overrides.json').read_text())[tag]
        env.update({k: os.path.expandvars(str(v)) for k, v in extra.items() if not k.startswith('_')})
        if base == 'kiss':
            cmd += kiss_args(env, RIGS[rig][1])
        return cmd, env
    rmin, rmax, rings, vfov = RIGS[rig]
    rolling = shutter == 'rolling'
    env = {'BENCH_MIN_RANGE': str(rmin), 'BENCH_MAX_RANGE': str(rmax),
           'BENCH_SCAN_LINES': str(rings), 'BENCH_VFOV_DEG': str(vfov),
           'BENCH_IGNORE_POINT_STAMPS': 'false' if rolling else 'true',
           'BENCH_DESKEW': 'true' if rolling else 'false'}
    if method in ('mola_lo', 'mola_lio'):
        env.update(MOLA_LIDAR_NAME='lidar1', MOLA_IMU_NAME='imu1',
                   MOLA_DESKEW_METHOD=('MotionCompensationMethod::IMU' if rolling
                                       else 'MotionCompensationMethod::None'))
        if method == 'mola_lio':
            env['MOLA_STATE_ESTIMATOR_YAML'] = str(CFG / 'mola-smoother-lidar-imu-only.yaml')
            # The configuration developed on the training split: on near-horizontal surfaces,
            # the side a view sees is the sign of its vertical component (needs the
            # gravity-aligned map frame of the LIO).
            env['MOLA_LOCALMAP_VIEW_DIRECTION_FILTER'] = \
                'ViewDirectionFilter::SurfaceSideGravityAligned'
        else:
            # The default estimator, without the rawlogs' wheel odometry (no other method reads it).
            env['MOLA_STATE_ESTIMATOR_YAML'] = str(CFG / 'mola-lo-simple-no-odometry.yaml')
        cmd = ['mola-lo-cli-rawlog', rawlog, '--lidar-sensor-label', 'lidar1',
               '--imu-sensor-label', 'imu1', '--output-tum-path', out_tum]
    elif method == 'kiss':
        cmd = ['mola-lidar-odometry-cli-kiss', '--input-rawlog', rawlog,
               '--lidar-sensor-label', 'lidar1', '--min-range', str(rmin),
               '--max-range', str(rmax), '--output-tum-path', out_tum]
        if not rolling:
            cmd.append('--no-deskew')
        # One thread, as for every method (KISS-ICP's default is every core of the CPU mask).
        cmd += ['--num-threads', '1']
    elif method == 'glim':
        cmd = ['mola-glim-cli', '-c', str(CFG / 'glim-bench.yaml'), '--input-rawlog', rawlog,
               '--num-threads', '1', '--output-tum-path', out_tum]
        # The JSON configuration of the install in use: the wrapper's default is the path of
        # the install it was built into, which a frozen copy does not follow.
        env['GLIM_CONFIG_PATH'] = os.path.expandvars(
            f'{LIO_WS}/mola_glim_wrapper/share/mola_glim_wrapper/glim-config')
    elif method == 'dlio':
        cmd = ['mola-dlio-cli', '-c', str(CFG / 'dlio-bench.yaml'), '--input-rawlog', rawlog,
               '--output-tum-path', out_tum]
    elif method == 'fastlio2':
        cmd = ['mola-fast-lio2-cli', '-c', str(CFG / 'fastlio2-bench.yaml'),
               '--input-rawlog', rawlog, '--output-tum-path', out_tum]
    elif method == 'ellipselio':
        cmd = ['mola-ellipse-lio-cli', '-c', str(CFG / 'ellipselio-bench.yaml'),
               '--input-rawlog', rawlog, '--output-tum-path', out_tum]
    elif method == 'trajlo':
        cmd = ['mola-traj-lo-cli', '-c', str(CFG / 'trajlo-bench.yaml'), '--input-rawlog',
               rawlog, '--output-tum-path', out_tum]
    else:
        raise ValueError(method)
    return cmd, env


def read_cores(path):
    out = []
    for part in Path(path).read_text().split(','):
        part = part.strip()
        if not part:
            continue
        a, _, b = part.partition('-')
        out.extend(range(int(a), int(b or a) + 1))
    return out


def headroom_gb():
    r = subprocess.run([str(TOOLS / 'memguard.sh')], capture_output=True, text=True)
    return float(r.stdout.strip() or 0)


def tree_rss_gb(root_pid):
    """Resident memory of a process and all its descendants."""
    children = {}
    rss = {}
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            st = (d / 'status').read_text()
        except OSError:
            continue
        pp = re.search(r'^PPid:\s+(\d+)', st, re.M)
        m = re.search(r'^VmRSS:\s+(\d+)', st, re.M)
        children.setdefault(int(pp.group(1)) if pp else 0, []).append(int(d.name))
        rss[int(d.name)] = int(m.group(1)) if m else 0
    total = 0
    stack = [root_pid]
    while stack:
        p = stack.pop()
        total += rss.get(p, 0)
        stack.extend(children.get(p, []))
    return total / 1048576


def batch_rss_gb(out_root):
    """Memory held by everything this effort runs: the LIO jobs writing below
    EFFORT_ROOT (any runner instance, any output tree), and the renders, map
    exports and figure jobs next to them. A LIO job counts as the larger of its
    resident size (all its processes) and the reservation it was launched with
    (<output>.resv), since jobs grow well after their launch."""
    needle = f'--output-tum-path\0{EFFORT_ROOT}/'.encode()
    others = [b'mvsim-dataset-gen', b'bench-map-export', b'make_banner', b'make_maps.py']
    jobs = {}
    total = 0.0
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            cmd = (d / 'cmdline').read_bytes()
            is_job = needle in cmd
            if not is_job and not any(n in cmd for n in others):
                continue
            m = re.search(r'^VmRSS:\s+(\d+)', (d / 'status').read_text(), re.M)
        except OSError:
            continue
        gb = int(m.group(1)) / 1048576 if m else 0.0
        if is_job:
            args = cmd.split(b'\0')
            tum = args[args.index(b'--output-tum-path') + 1].decode()
            jobs[tum] = jobs.get(tum, 0.0) + gb
        else:
            total += gb
    for tum, gb in jobs.items():
        try:
            resv = float(Path(tum).with_suffix('.resv').read_text())
        except (OSError, ValueError):
            resv = 0.0
        total += max(gb, resv)
    return total


def busy_cores(out_root):
    """Cores pinned by live LIO jobs of this effort (any runner instance), from
    their launch on: the job shell is pinned too, and quotes the output path."""
    needles = [f'--output-tum-path\0{EFFORT_ROOT}/'.encode(),
               f"'--output-tum-path' '{EFFORT_ROOT}/".encode()]
    out = set()
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            cmd = (d / 'cmdline').read_bytes()
            if not any(n in cmd for n in needles):
                continue
            m = re.search(r'^Cpus_allowed_list:\s+(\S+)', (d / 'status').read_text(), re.M)
        except OSError:
            continue
        if m and '-' not in m.group(1) and ',' not in m.group(1):
            out.add(int(m.group(1)))
    return out


# Hard per-job memory limit [GB]: a job whose resident memory grows past it is
# killed, so one job cannot take the machine's memory, and restarted with twice
# the limit, up to JOB_LIMIT_MAX_GB. Traj-LO keeps its whole map and starts with
# a tighter limit. No completed run used more than 10 GB: the maximum keeps a
# margin above that, and a run that needs more counts as a memory failure.
JOB_LIMIT_GB = {'trajlo': 4.0}
JOB_LIMIT_MIN_GB = 6.0
JOB_LIMIT_MAX_GB = 16.0
# Hard wall-time limit per job [s], so a hung job does not hold a core forever. It
# guards against hangs only: some single-threaded runs of the slowest method on the
# densest sensor take about a day on a busy machine, and a run must not fail for that.
JOB_WALL_LIMIT_S = 48 * 3600.0


def job_limit_gb(method, rawlog_gb, kills=0):
    """Memory limit of a job whose earlier attempts were killed `kills` times."""
    base = method.split('+', 1)[0]
    limit = JOB_LIMIT_GB.get(base, max(JOB_LIMIT_MIN_GB, 2.0 * (1.9 * rawlog_gb + 0.4)))
    return min(JOB_LIMIT_MAX_GB, limit * 2**kills)


def memory_kills(tum):
    """Limits [GB] at which earlier attempts of a job were killed, one per process."""
    f = tum.with_suffix('.memlimit')
    if not f.exists():
        return []
    kills = {}
    for i, line in enumerate(f.read_text().splitlines()):
        m = re.search(r'limit ([\d.]+) GB', line)
        if m:
            pid = re.search(r'pid=(\d+)', line)
            kills[pid.group(1) if pid else f'line{i}'] = float(m.group(1))
    return list(kills.values())


def retries_exhausted(tum):
    kills = memory_kills(tum)
    return bool(kills) and max(kills) >= JOB_LIMIT_MAX_GB


def limit_kill(tum):
    """'memory' or 'time' if the last attempt of a job was killed at the runner's memory or
    wall-time limit (the newer of the two markers), None if it ended on its own or crashed."""
    tm = tum.with_suffix('.time')
    if not tm.exists() or 'signal 9' not in tm.read_text():
        return None
    marks = [(f.stat().st_mtime, kind)
             for kind, f in (('memory', tum.with_suffix('.memlimit')),
                             ('time', tum.with_suffix('.timeout')))
             if f.exists()]
    return max(marks)[1] if marks else None


def time_limit_of_kill(tum):
    """Wall-time limit [s] at which the last attempt of a job was killed."""
    m = re.search(r'limit ([\d.]+) s', tum.with_suffix('.timeout').read_text())
    return float(m.group(1)) if m else float('inf')


def proc_age_s(stat_text):
    start_ticks = int(stat_text.rsplit(')', 1)[1].split()[19])
    uptime = float(Path('/proc/uptime').read_text().split()[0])
    return uptime - start_ticks / os.sysconf('SC_CLK_TCK')


def watchdog():
    """Kills any live job of this effort above its memory or wall-time limit;
    returns the kills."""
    killed = []
    needle = f'--output-tum-path\0{EFFORT_ROOT}/'.encode()
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            raw = (d / 'cmdline').read_bytes()
            if needle not in raw:
                continue
            args = [x.decode() for x in raw.split(b'\0') if x]
            if args[0].endswith('/time'):
                continue
            m = re.search(r'^VmRSS:\s+(\d+)', (d / 'status').read_text(), re.M)
            age = proc_age_s((d / 'stat').read_text())
        except OSError:
            continue
        rss = int(m.group(1)) / 1048576 if m else 0.0
        tum = Path(args[args.index('--output-tum-path') + 1])
        rawlog = next((Path(x) for x in args if x.endswith('.rawlog')), None)
        size = rawlog.stat().st_size / 1e9 if rawlog and rawlog.exists() else 1.0
        limit = job_limit_gb(tum.stem, size, len(memory_kills(tum)))
        if rss > limit:
            try:
                os.kill(int(d.name), 9)
            except OSError:
                continue
            # Appended, one line per attempt: the next attempt gets a larger limit.
            with open(tum.with_suffix('.memlimit'), 'a') as fh:
                fh.write(f'pid={d.name} killed at {rss:.1f} GB resident '
                         f'(limit {limit:.1f} GB)\n')
            killed.append(f'{tum} at {rss:.1f} GB (limit {limit:.1f})')
        elif age > JOB_WALL_LIMIT_S:
            try:
                os.kill(int(d.name), 9)
            except OSError:
                continue
            tum.with_suffix('.timeout').write_text(
                f'killed after {age:.0f} s (limit {JOB_WALL_LIMIT_S:.0f} s)\n')
            killed.append(f'{tum} after {age:.0f} s (time limit)')
    return killed


_PEAKS = None


def known_peaks():
    """Highest resident size [GB] seen for (variant, seq dir, method), from the
    .time files of every run of this effort (runs, tuning and archives)."""
    global _PEAKS
    if _PEAKS is None:
        _PEAKS = {}
        for root, _dirs, files in os.walk(EFFORT_ROOT):
            for name in files:
                if not name.endswith('.time'):
                    continue
                try:
                    m = re.search(r'maxrss_kb=(\d+)', (Path(root) / name).read_text())
                except OSError:
                    continue
                if not m:
                    continue
                parts = Path(root).parts
                key = (parts[-2], parts[-1], name[:-len('.time')])
                _PEAKS[key] = max(_PEAKS.get(key, 0.0), int(m.group(1)) / 1048576)
    return _PEAKS


def job_mem_gb(job, out_root):
    """Peak RSS estimate: the highest peak seen for this method (or its base
    method) on this sequence and variant (or rig), with a margin; else from the
    rawlog size (the readers keep the whole decompressed log). A job killed for
    memory before reserves its whole new limit."""
    variant, seq, method = job
    f = DATA / seq / f'{variant}.rawlog'
    size = f.stat().st_size / 1e9 if f.exists() else 1.0
    world, traj = seq.split('/')
    kills = memory_kills(out_root / variant / f'{world}__{traj}' / f'{method}.tum')
    if kills:
        return job_limit_gb(method, size, len(kills))
    peaks = known_peaks()
    sdir = f'{world}__{traj}'
    base = method.split('+', 1)[0]
    rig = variant.split('-', 1)[0]
    seen = [g for (v, s, m), g in peaks.items() if s == sdir and m == method and v == variant]
    if not seen:
        seen = [g for (v, s, m), g in peaks.items()
                if s == sdir and m.split('+', 1)[0] == base and v.split('-', 1)[0] == rig]
    est = 1.9 * size + 0.4
    return max(est, 1.15 * max(seen)) if seen else est


def tum_of(job, out_root):
    variant, seq, method = job
    world, traj = seq.split('/')
    return out_root / variant / f'{world}__{traj}' / f'{method}.tum'


def is_running(job, out_root):
    variant, seq, method = job
    world, traj = seq.split('/')
    tum = out_root / variant / f'{world}__{traj}' / f'{method}.tum'
    # pgrep takes a regular expression: escape the '+' of tuned method names.
    pattern = re.sub(r'([.+*?()\[\]{}|^$\\])', r'\\\1', f'--output-tum-path {tum}')
    r = subprocess.run(['pgrep', '-f', '--', pattern], capture_output=True)
    return r.returncode == 0


def run_job(job, core, out_root, force):
    variant, seq, method = job
    world, traj = seq.split('/')
    rig, _noise, shutter = variant.split('-')
    rawlog = DATA / world / traj / f'{variant}.rawlog'
    odir = out_root / variant / f'{world}__{traj}'
    odir.mkdir(parents=True, exist_ok=True)
    tum = odir / f'{method}.tum'
    if tum.exists() and tum.stat().st_size > 0 and not force:
        return None
    if not rawlog.exists():
        return 'missing rawlog'
    cmd, env = method_cmd(method, str(rawlog), str(tum), rig, shutter)
    exports = ' '.join(f'{k}={shlex.quote(str(v))}' for k, v in {**ONE_THREAD, **env}.items())
    timef = odir / f'{method}.time'
    log = odir / f'{method}.log'
    # Progress bars redraw with \r: split them into lines, drop them, and keep
    # the first and last 300 lines only, so that logs stay small.
    shell = (f'set -o pipefail; {ENV_SETUP} && cd {odir} && '
             f'/usr/bin/time -f "wall=%e user=%U sys=%S maxrss_kb=%M exit=%x" -o {timef} '
             f'env {exports} taskset -c {core} nice -n 10 '
             + ' '.join(f"'{c}'" for c in cmd) +
             " 2>&1 | tr '\\r' '\\n' | grep -av 'ETA=' | "
             "awk 'NR<=300{print;next}{b[NR%300]=$0;n=NR}END{if(n>300){print \"[...]\";"
             "for(i=n-299;i<=n;i++){if(i>300)print b[i%300]}}}' > " + str(log))
    # Pinned from the start, so that other runners see the core as taken.
    return subprocess.Popen(['bash', '-c', shell], start_new_session=True,
                            preexec_fn=lambda: os.sched_setaffinity(0, {core}))


def mem_cap_gb(a):
    """The batch memory cap: from --mem-cap-file if it holds a number, so that
    it can change while the runner works, else --mem-cap-gb."""
    if a.mem_cap_file:
        try:
            return float(Path(a.mem_cap_file).read_text().strip())
        except (OSError, ValueError):
            pass
    return a.mem_cap_gb


@contextlib.contextmanager
def launch_lock():
    with open(EFFORT_ROOT / '.launch.lock', 'a') as fh:
        fcntl.flock(fh, fcntl.LOCK_EX)
        yield


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variants', required=True)
    ap.add_argument('--methods', required=True)
    ap.add_argument('--seqs', default='', help='world/traj,... (default: all)')
    ap.add_argument('--out', default=str(EFFORT_ROOT / 'runs'))
    ap.add_argument('--cores-file', default=str(HERE / 'cores.txt'))
    ap.add_argument('--max-jobs', type=int, default=16)
    ap.add_argument('--mem-cap-gb', type=float, default=20.0,
                    help='total RSS of this batch')
    ap.add_argument('--mem-cap-file', default='',
                    help='file holding the cap in GB, read at each launch (overrides --mem-cap-gb)')
    ap.add_argument('--min-headroom-gb', type=float, default=15.0)
    ap.add_argument('--launch-interval', type=float, default=60.0,
                    help='minimum seconds between job starts (load peaks never coincide)')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()

    out_root = Path(a.out)
    seqs = [s for s in a.seqs.split(',') if s] or sorted(
        f'{p.parent.parent.name}/{p.parent.name}'
        for p in DATA.glob('*/*/gt_200hz.tum'))
    # Longest sequences first, so that the batch does not end on a long tail.
    def duration(seq):
        g = DATA / seq / 'gt_200hz.tum'
        return g.stat().st_size if g.exists() else 0
    # Traj-LO needs real per-point times: rolling shutter only.
    # Variants in the given order (priority), longest sequences first within each.
    # "method@variant" restricts a method to one variant (e.g. tuning candidates).
    def wanted(m, v):
        name, _, only = m.partition('@')
        return (not only or only == v) and not (
            name.startswith('trajlo') and v.endswith('-global'))
    jobs = [(v, sq, m.partition('@')[0]) for v in a.variants.split(',')
            for sq in sorted(seqs, key=duration, reverse=True) for m in a.methods.split(',')
            if wanted(m, v)]
    def done(job):
        v, sq, m = job
        w, t = sq.split('/')
        f = out_root / v / f'{w}__{t}' / f'{m}.tum'
        if a.force:
            return False
        if f.exists() and f.stat().st_size > 0:
            return True
        tm = f.with_suffix('.time')
        if not tm.exists() or tm.stat().st_size == 0 or 'signal 15' in tm.read_text():
            return False  # never run, running, or stopped by the operator
        # A memory kill earns another attempt, at a doubled limit; a time kill one at a
        # larger wall-time limit than the one it was killed at.
        kill = limit_kill(f)
        if kill == 'memory':
            return retries_exhausted(f)
        if kill == 'time':
            return time_limit_of_kill(f) >= JOB_WALL_LIMIT_S
        return True
    running = {}  # core -> (job, Popen, t0, reserved_gb)
    pending = [j for j in jobs if not done(j)]
    print(f'{len(jobs)} jobs, {len(pending)} to run', flush=True)
    last_launch = 0.0
    while pending or running:
        for k in watchdog():
            print(f'LIMIT: killed {k}', flush=True)
        for core, (job, p, t0, _res) in list(running.items()):
            if p.poll() is not None:
                print(f'done {"/".join(job)} core={core} rc={p.returncode} '
                      f'{time.time() - t0:.0f}s', flush=True)
                tum_of(job, out_root).with_suffix('.resv').unlink(missing_ok=True)
                del running[core]
        # One launch at a time over all runner instances, so that two of them
        # never take the same free core.
        with launch_lock():
            busy = busy_cores(out_root)
            cores = [c for c in read_cores(a.cores_file) if c not in running and c not in busy]
            launched = False
            if pending and cores and len(running) < a.max_jobs and \
                    time.time() - last_launch >= a.launch_interval and \
                    headroom_gb() > a.min_headroom_gb:
                # Memory reserved by the running jobs: the larger of their
                # estimate and their current resident size.
                # Includes jobs of this output tree started by other runner instances.
                reserved = batch_rss_gb(out_root)
                cap = mem_cap_gb(a)
                # First fit: the first pending job whose estimate fits the cap.
                for i, job in enumerate(pending):
                    need = job_mem_gb(job, out_root)
                    if need > cap:
                        # It would never fit: left for a run with a larger cap.
                        pending.pop(i)
                        print(f'defer {"/".join(job)}: needs {need:.1f}GB, above the cap',
                              flush=True)
                        launched = True
                        break
                    if reserved + need > cap:
                        continue
                    pending.pop(i)
                    if done(job):
                        # Finished meanwhile, e.g. an orphan killed by the watchdog.
                        launched = True
                        break
                    if is_running(job, out_root):
                        print(f'skip {"/".join(job)}: already running', flush=True)
                        launched = True
                        break
                    p = run_job(job, cores[0], out_root, a.force)
                    if isinstance(p, subprocess.Popen):
                        tum_of(job, out_root).with_suffix('.resv').write_text(f'{need:.2f}\n')
                        # Hold the launch lock until the job's processes show up, so
                        # that the next launch (any runner) counts its reservation.
                        time.sleep(2)
                        last_launch = time.time()
                        running[cores[0]] = (job, p, time.time(), need)
                        print(f'start {"/".join(job)} core={cores[0]} est={need:.1f}GB '
                              f'reserved={reserved + need:.1f}GB', flush=True)
                    elif p:
                        print(f'skip {"/".join(job)}: {p}', flush=True)
                    launched = True
                    break
        time.sleep(5)
    print('ALL DONE', flush=True)


if __name__ == '__main__':
    sys.exit(main())
