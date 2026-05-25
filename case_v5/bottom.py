"""v5 bottom case wrapper for cad skill."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from case_v5 import make_bottom


def gen_step():
    return make_bottom()


if __name__ == "__main__":
    p = gen_step()
    print(f"Volume: {p.volume:.0f} mm³")
    print(f"BBox: {p.bounding_box().size}")
