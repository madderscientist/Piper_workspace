import os
from pathlib import Path

# Tests use the tracked four-arm examples, independent of this machine's config.
os.environ['PIPER_CONFIG_DIR'] = str(Path(__file__).parent / 'config/examples')
