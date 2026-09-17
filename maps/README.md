scan_voxel_008.pcd is a simulation-side derivative of
navigationros2/src/mapping_and_location/small_point_lio/pcd/scan.pcd.
It keeps the first finite point in each 0.08 m voxel using sparse integer keys,
without rotating, translating, or cropping the source map. The source is unchanged.
This avoids the large memory cost when PCL's dense voxel index overflows on the
source map's unusually large coordinate bounds. Regenerate it when replacing scan.pcd.
This reduces resource usage; it does not correct any drift in the source map.

From /home/dengjiaxi/simulation_seu, regenerate after replacing the source:

```bash
python3 navigationsim/maps/downsample_scan.py \
  navigationros2/src/mapping_and_location/small_point_lio/pcd/scan.pcd \
  navigationsim/maps/scan_voxel_008.pcd
```
