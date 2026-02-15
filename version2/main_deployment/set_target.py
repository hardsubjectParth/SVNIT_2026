import json
import argparse
from pathlib import Path

TARGET_FILE = Path("/home/pi/mission/target_coords.json")

parser = argparse.ArgumentParser()
parser.add_argument("--lat", type=float, required=True)
parser.add_argument("--lon", type=float, required=True)
parser.add_argument("--alt", type=float, default=2.0)
args = parser.parse_args()

TARGET_FILE.write_text(json.dumps({
    "lat": args.lat,
    "lon": args.lon,
    "alt": args.alt
}, indent=2))

print("✅ Target set:", args.lat, args.lon, args.alt)
