"""colcon を使わずに pytest だけで実行できるよう、2つのパッケージのソースを import できるようにする."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
TASK_ROOT = os.path.join(os.path.dirname(SIM_ROOT), "elevator_task")
for path in (HERE, SIM_ROOT, TASK_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)
