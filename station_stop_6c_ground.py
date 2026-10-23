"""Cancellable, process-isolated CARLA ground queries for the 6c editor.

A CARLA RPC may block inside a native extension. Keeping it in a subprocess
protects the Pygame thread even when that native call holds the Python GIL.
The worker is read-only: it never ticks or changes the CARLA world.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time


class GroundQuery:
    def __init__(self, timeout_s=5., worker_path=None):
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError('ground query timeout must be positive')
        self.worker_path = Path(worker_path or __file__).resolve()
        self.pending = None
        self.retired = []

    def _reap(self):
        for process in list(self.retired):
            if process.poll() is not None:
                process.communicate()
                self.retired.remove(process)

    def start(self, request):
        self._reap()
        if self.pending is not None:
            raise ValueError('正在查询高程；Esc 取消后可重新选点')
        if self.retired:
            raise ValueError('正在结束上一高程查询，请稍后再点击')
        # No shell. Capture mode/reference geometry now, not when the reply arrives.
        request = dict(request)
        process = subprocess.Popen(
            [sys.executable, '-B', str(self.worker_path), json.dumps(request, allow_nan=False)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, start_new_session=True)
        self.pending = dict(process=process, request=request, started=time.monotonic())

    def cancel(self):
        if self.pending is None:
            return None
        pending, self.pending = self.pending, None
        process = pending['process']
        if process.poll() is None:
            try:
                process.kill()
            except ProcessLookupError:
                pass
        self.retired.append(process)
        return pending['request']

    def poll(self):
        self._reap()
        if self.pending is None:
            return None
        item = self.pending
        process, request = item['process'], item['request']
        if process.poll() is None:
            if time.monotonic() - item['started'] < self.timeout_s:
                return None
            self.cancel()
            return dict(ok=False, request=request,
                        error='高程查询超过 {:.1f}s，CARLA 未及时响应；未放置该点。请检查连接后重试。'.format(self.timeout_s))
        self.pending = None
        stdout, stderr = process.communicate()
        try:
            if process.returncode != 0:
                raise ValueError('ground worker exited with code ' + str(process.returncode))
            # CARLA may emit a diagnostic before our final protocol line.
            payload = json.loads(stdout.strip().splitlines()[-1])
            if not isinstance(payload, dict) or not isinstance(payload.get('ok'), bool):
                raise ValueError('invalid ground response')
            if payload['ok']:
                p = payload['point']
                if not all(math.isfinite(float(p[k])) for k in ('x', 'y', 'z')):
                    raise ValueError('nonfinite ground point')
                if abs(p['x'] - request['x']) > 1e-7 or abs(p['y'] - request['y']) > 1e-7:
                    raise ValueError('ground response changed click coordinates')
                if not isinstance(payload['evidence'], dict):
                    raise ValueError('missing ground evidence')
            payload['request'] = request
            return payload
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            return dict(ok=False, request=request,
                        error='高程查询未返回有效结果：' + (stderr.strip() or str(exc))[:240])

    def close(self):
        self.cancel()
        for process in self.retired:
            if process.poll() is None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
            try:
                process.communicate(timeout=.5)
            except subprocess.TimeoutExpired:
                # Do not hold the UI open for a broken native client at shutdown.
                pass
        self.retired.clear()


def query_ground(request):
    import carla
    client = carla.Client(request['host'], int(request['port']))
    client.set_timeout(float(request.get('rpc_timeout_s', 2.)))
    world = client.get_world()
    map_obj = world.get_map()
    if map_obj.name != request['map_name']:
        raise ValueError('CARLA 地图已变化，请重新打开编辑器')
    if hashlib.sha256(map_obj.to_opendrive().encode()).hexdigest() != request['map_sha256']:
        raise ValueError('CARLA 地图几何已变化，请重新打开编辑器')
    x, y, z = request['x'], request['y'], request['reference_z']
    hits = world.cast_ray(carla.Location(x=x, y=y, z=z + 8),
                          carla.Location(x=x, y=y, z=z - 3))
    candidates = [h for h in hits if abs(h.location.z - z) < 2]
    if not candidates:
        raise ValueError('该位置未检测到路面附近的表面；未放置该点，请重新选点')
    hit = min(candidates, key=lambda h: abs(h.location.z - z))
    label = str(hit.label).rsplit('.', 1)[-1]
    if request['mode'] == 'goal' and label.lower() in ('sidewalks', 'buildings', 'walls', 'fences'):
        raise ValueError('停车目标命中人行道或建筑，请放在站台旁路面')
    return dict(ok=True, point=dict(x=x, y=y, z=hit.location.z),
                evidence=dict(source='carla.cast_ray', label=label, reference_z=z, hit_z=hit.location.z))


if __name__ == '__main__':
    try:
        response = query_ground(json.loads(sys.argv[1]))
    except Exception as exc:
        response = dict(ok=False, error=str(exc)[:400])
    print(json.dumps(response, ensure_ascii=False, allow_nan=False), flush=True)
