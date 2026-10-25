#!/usr/bin/env python3
"""Run MonoGS's mapper as a thread (WSL2 has no CUDA IPC). Idempotent. Usage: patch_threads.py <MonoGS>/slam.py"""
import sys
path = sys.argv[1]
s = open(path).read()
if "_CopyQueue" in s:
    print("already patched:", path); sys.exit(0)
HELPER = '''import copy as _copy
import queue as _queue
import threading as _threading


class _CopyQueue(_queue.Queue):
    def put(self, item, block=True, timeout=None):
        super().put(_copy.deepcopy(item), block, timeout)

'''
edits = [
    ("        frontend_queue = mp.Queue()\n", "        frontend_queue = _CopyQueue()\n"),
    ("        backend_queue = mp.Queue()\n", "        backend_queue = _CopyQueue()\n"),
    ("        backend_process = mp.Process(target=self.backend.run)\n",
     "        backend_process = _threading.Thread(target=self.backend.run, daemon=True)\n"),
]
for old, new in edits:
    if s.count(old) != 1:
        sys.exit("expected exactly one occurrence of %r" % old.strip())
    s = s.replace(old, new)
anchor = "\n\nclass SLAM:\n"
if s.count(anchor) != 1:
    sys.exit("could not find class SLAM")
s = s.replace(anchor, "\n\n" + HELPER + "\nclass SLAM:\n")
open(path, "w").write(s); print("patched:", path)
