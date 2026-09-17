"""Create a sparse-voxel simulation map without changing source coordinates.

Usage: python3 downsample_scan.py input.pcd output.pcd
Only binary float32 x/y/z/intensity PCD is supported.
"""
import sys
from pathlib import Path
import numpy as np

source, output = map(Path, sys.argv[1:3])
if source.resolve() == output.resolve():
    raise ValueError("Output must differ from the original map")
header = {}
with source.open("rb") as stream:
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("Missing PCD DATA header")
        fields = line.decode().strip().split()
        if fields and not fields[0].startswith("#"):
            header[fields[0]] = fields[1:]
        if fields[:1] == ["DATA"]:
            offset = stream.tell()
            break
for key, expected in {"DATA": ["binary"], "FIELDS": ["x", "y", "z", "intensity"],
                      "SIZE": ["4"] * 4, "TYPE": ["F"] * 4,
                      "COUNT": ["1"] * 4}.items():
    if header.get(key) != expected:
        raise ValueError(f"Unsupported PCD {key}")
cloud = np.memmap(source, dtype="<f4", mode="r", offset=offset).reshape(-1, 4)
if len(cloud) != int(header["POINTS"][0]):
    raise ValueError("PCD point count mismatch")
chunks = []
for start in range(0, len(cloud), 250000):
    points = np.asarray(cloud[start:start + 250000])
    points = points[np.isfinite(points[:, :3]).all(axis=1)]
    _, indices = np.unique(np.floor(points[:, :3] / .08).astype(np.int64),
                           axis=0, return_index=True)
    chunks.append(points[indices].copy())
points = np.concatenate(chunks)
_, indices = np.unique(np.floor(points[:, :3] / .08).astype(np.int64),
                       axis=0, return_index=True)
points = points[indices]
temporary = output.with_suffix(output.suffix + ".tmp")
with temporary.open("wb") as stream:
    stream.write(("VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                  "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n"
                  "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
                  % (len(points), len(points))).encode())
    stream.write(points.astype("<f4").tobytes())
temporary.replace(output)
print(f"{len(cloud)} -> {len(points)} points: {output}")
