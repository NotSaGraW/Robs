# Result: test_sensor_360.py

**Date:** 2026-05-25
**Result: PASS**

## Summary
- Total detections: 1203
- Pass rate (err < 0.05m): 100.0%
- Average error: 0.0250m
- Max error: 0.0349m

## Formula verified
`detected_world_pos = sensor_world_matrix × detected_point_local`

## Metric
Distance to nearest wall **plane** (not segment centre).

Formula accurate. Communication architecture implementation unblocked.
