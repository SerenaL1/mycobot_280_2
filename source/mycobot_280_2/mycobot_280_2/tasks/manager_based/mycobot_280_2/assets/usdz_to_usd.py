"""
Convert USDZ to USD for Isaac Lab

Place this script in the same directory as your tray.usdz file and run:
python convert_usdz_to_usd.py
"""

from pxr import Usd

# Input USDZ file
usdz_path = "tray.usdz"
# Output USD file
usd_path = "tray.usd"

# Open the USDZ stage and export as USD
stage = Usd.Stage.Open(usdz_path)
stage.Export(usd_path)

print(f"Converted {usdz_path} to {usd_path}")