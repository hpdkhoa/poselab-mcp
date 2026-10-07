"""One labelled contact sheet from the rendered views."""
import io
from pathlib import Path

from PIL import Image, ImageDraw


def sheet(images):
    """The views two across, each labelled as a Blender view; saved beside them. Returns (path, PNG bytes)."""
    tiles = []
    for im in images:
        t = Image.open(im["path"]).convert("RGB")
        d = ImageDraw.Draw(t)
        v = im["view"]
        label = "Blender view: %s" % ("player's eye" if v == "eye" else v + " side" if v in ("left", "right") else v)
        d.rectangle((0, 0, 8 + 6 * len(label), 20), fill=(0, 0, 0))
        d.text((5, 4), label, fill=(255, 220, 0))
        tiles.append(t)
    w, h = tiles[0].size
    cols = 2 if len(tiles) > 1 else 1
    rows = (len(tiles) + cols - 1) // cols
    out = Image.new("RGB", (w * cols, h * rows))
    for i, t in enumerate(tiles):
        out.paste(t, ((i % cols) * w, (i // cols) * h))
    path = Path(images[0]["path"]).parent / "sheet.png"
    out.save(path)
    buf = io.BytesIO()
    out.save(buf, "PNG")
    return str(path), buf.getvalue()
