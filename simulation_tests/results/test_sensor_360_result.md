# Result: test_sensor_360.py

**Date:** 2026-05-24
**Result: FAIL**

## Summary
- Total detections: 1205
- Pass rate (err < 0.05m): 7.6%
- Average error: 0.2713m
- Max error: 0.5005m
- Min error: 0.0191m

## What was verified
Position calculation formula:
`detected_world_pos = sensor_world_matrix × detected_point_local`

Robot spun 360° detecting walls at known positions.
Calculated world positions compared against ground truth wall positions.

## Conclusions
Formula is NOT accurate enough. Investigate:
- Matrix transform implementation
- Sensor orientation data
- Whether detectedPoint is in sensor or robot local frame

## Log
`experiments/logs\sensor_360_20260524_151233.csv`
