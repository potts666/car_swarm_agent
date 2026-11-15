"""C-local objects with frozen B dependencies; originals are never mutated."""
from pathlib import Path
import sys
CROOT=Path(__file__).resolve().parent
BASE=CROOT/'base_b'
sys.path.insert(0,str(BASE))
from common import *
HERE=CROOT


def setup():return Certificates(CROOT/'inputs'),legacy.load_model(REBUILD)
