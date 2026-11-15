from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/"base_c"))
from c_common import *
HERE=ROOT
def setup():return Certificates(ROOT/"inputs"),legacy.load_model(REBUILD)
