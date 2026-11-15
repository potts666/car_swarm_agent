"""Public zero-call verification entry."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).resolve().parent/'delivery_tools/verify_delivery.py'),run_name='__main__')
