# Object Detection

The `ObjectDetector` class (`src/vision/objects/ObjectDetector.cpp`) detects and tracks non-fiducial competition props, including mallets, rock picks, and water bottles.

---

## 1. Deep Learning Pipeline: LibTorch YOLO

Unlike fiducial markers with geometric patterns, natural ground props require convolutional neural networks for robust classification under variable desert lighting.

```
[Raw Camera Image] (cv::Mat, 1280x720)
        |
        v
[Preprocessing] (yolomodel::pytorch::PyTorchInterpreter)
 - Resize / Letterbox to 640x640
 - Normalize channels to [0.0, 1.0]
 - Convert to CUDA FloatTensor [1, 3, 640, 640]
        |
        v
[Inference on GPU] (LibTorch torch::jit::load)
 - Model: OBJECTDETECT_TORCH_MODEL (.pt TorchScript)
        |
        v
[Post-Processing]
 - Confidence Filter (OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE)
 - Non-Maximum Suppression (cv::dnn::NMSBoxes)
        |
        v
[Tracking & Temporal Validation]
 - OpenCV CSRT / KCF MultiTracker
 - BBOX_MIN_LIFETIME_THRESHOLD Filter
 - BBOX_MIN_SCREEN_PERCENTAGE Filter
        |
        v
[3D Point Cloud Geolocation]
 - GeolocateBox() against ZED Point Cloud
        |
        v
[objectdetectutils::Object Struct]
```

---

## 2. Target Classification and Parsing

The system detects three primary competition classes:
- **Mallet**: Orange rubber mallet (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::MALLET`).
- **Water Bottle**: 1-liter plastic bottle (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::WATERBOTTLE`).
- **Rock Pick**: Geologist rock hammer (`manifest::Autonomy::AUTONOMYWAYPOINTTYPES::ROCKPICK`).

When evaluating detections in `ObjectDetectionChecker::IdentifyTargetObject()`:
1. Active detections are matched against the target class requested by the current waypoint leg.
2. If multiple instances appear, the candidate with the highest screen area percentage is selected.
3. Candidate objects must exceed `constants::BBOX_MIN_LIFETIME_THRESHOLD` to eliminate transient false positives.

---

## 3. 3D Geolocation Integration

Because competition props vary in dimensions and orientation, estimating distance via 2D pinhole trigonometry is prone to error. The `ObjectDetector` resolves physical location by pairing 2D bounding boxes with the ZED 3D point cloud:

1. Bounding box center coordinates $(u_c, v_c)$ are extracted from the detection.
2. The coordinate is passed to `geoloc::GeolocateBox()` along with the synchronized `CV_32FC4` point cloud matrix and the fused rover pose.
3. `GeolocateBox()` queries a 5x5 neighborhood around $(u_c, v_c)$, sorts the depth values, and computes the 20th percentile surface depth to isolate the object face from the desert ground behind it.
4. The localized 3D point $(X_c, Y_c, Z_c)$ is rotated by the rover compass heading and translated by the rover UTM position, generating an absolute `geoops::Waypoint`.

---

## 4. Usage in State Machine

During mission execution:
- In `eNavigating` or `eSearchPattern`, `ObjectDetectionChecker` monitors for target detections.
- Upon confirming a valid object, the state machine triggers `Event::eObjectSeen` and transitions to `eApproachingObject`.
- The rover visual-servos toward the object until distance drops below `constants::APPROACH_OBJECT_PROXIMITY_THRESHOLD`.
- The state machine triggers `Event::eReachedObject`, transitioning to `eVerifyingObject` to halt, confirm the detection hit-rate over time, and signal the C2 station.
