import os
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent
BASE_DIR = os.path.dirname(os.path.realpath(__file__))
print(ROOT_DIR)