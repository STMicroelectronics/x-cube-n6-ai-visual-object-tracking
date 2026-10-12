# Build Options

Some features are enabled using build options or by using `app_config.h`:

- [Camera Orientation](#camera-orientation)
- [Tracking Confidence Thresholds](#tracking-confidence-thresholds)

This documentation explains those features and how to modify them.

## Camera Orientation

Cameras allow flipping the image along two axes.

- `CMW_MIRRORFLIP_MIRROR`: Selfie mode
- `CMW_MIRRORFLIP_FLIP`: Flip upside down
- `CMW_MIRRORFLIP_FLIP_MIRROR`: Flip both axes
- `CMW_MIRRORFLIP_NONE`: Default

1. Open [app_config.h](../Inc/app_config.h).

2. Change the `CAMERA_FLIP` define:
```c
/* Defines: CMW_MIRRORFLIP_NONE; CMW_MIRRORFLIP_FLIP; CMW_MIRRORFLIP_MIRROR; CMW_MIRRORFLIP_FLIP_MIRROR; */

#define CAMERA_FLIP CMW_MIRRORFLIP_NONE
```

## Tracking Confidence Thresholds

The tracker uses several criteria to assess how reliably it is following the target. Among them, the Peak-to-Sidelobe Ratio (PSR) of the response map is a key indicator. Two thresholds in `app_config.h` control the transitions between tracking states:

- `PSR_WARN_THRESHOLD` : below this value the tracker enters a warning state — the bounding box turns orange to signal reduced confidence
- `PSR_LOST_THRESHOLD` : below this value the target is considered lost

Note that PSR is not the only criterion; other factors such as a sudden change in estimated target size can also trigger the warning or lost state.

1. Open [app_config.h](../Inc/app_config.h).

2. Adjust the thresholds to your use case:
```c
#define PSR_LOST_THRESHOLD  8.0f
#define PSR_WARN_THRESHOLD 15.0f
```
