"""Public zero-call verification entry."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).resolve().parent/'delivery_tools/verify_delivery.py'),run_name='__main__')
