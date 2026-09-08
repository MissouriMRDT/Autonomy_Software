# Perception Subsystem

The Perception subsystem processes optical and depth imagery to identify mission targets (ArUco fiducial tags, competition props) and map physical obstacles across the terrain.

---

## 1. Algorithmic Architecture

The perception pipeline integrates deep learning models, classical computer vision, stereoscopic depth mapping, and geometric projection:

```
[Raw Camera Stream] (ZED 2i / SIM WebRTC: 720p/1080p)
        |
        +-----------------------------------+
        |                                   |
        v                                   v
[TagDetector Pipeline]            [ObjectDetector Pipeline]
 - OpenCV ArUco (DICT_4X4_50)      - LibTorch YOLO (.pt on CUDA)
 - LibTorch YOLO Tag Fallback      - Non-Maximum Suppression (NMS)
 - CSRT / KCF Bounding Box Track   - CSRT / KCF Bounding Box Track
        |                                   |
        +-----------------+-----------------+
                          |
                          v
               [Target Identification]
                - Lifetime Thresholding (BBOX_MIN_LIFETIME_THRESHOLD)
                - Screen Area Filtering (BBOX_MIN_SCREEN_PERCENTAGE)
                - Target ID Matching (IdentifyTargetMarker / IdentifyTargetObject)
                          |
                          v
               [Geolocation Engine]
                - GeolocateBox() (src/util/vision/Geolocate.hpp)
                - 3D Point Cloud Neighborhood Sampling
                - Statistical 20th Percentile Depth Isolation
                - Monocular Ground Plane Raycast Fallback
                - UTM Coordinate Transformation via Rover Pose
                          |
                          v
            [Global Waypoint Output] (geoops::Waypoint)
```

---

## 2. Detection Subsystems

### A. AR Tag Detection (`TagDetector.cpp`)
Fiducial marker detection operates through a dual-path pipeline:
1. **Classical OpenCV ArUco**:
   - Uses `cv::aruco::detectMarkers` with dictionary `DICT_4X4_50`.
   - Employs sub-pixel corner refinement (`cv::aruco::CORNER_REFINE_SUBPIX` or `CORNER_REFINE_CONTOUR`) up to `TAGDETECT_MAINCAM_CORNER_REFINE_MAX_ITER`.
   - Inverted marker detection can be enabled via `TAGDETECT_MAINCAM_DETECT_INVERTED_MARKER` for handling low-contrast lighting conditions.
2. **LibTorch YOLO Tag Fallback**:
   - When distance or glare degrades high-frequency corner contrast, classical ArUco fails.
   - An optional YOLO model (`TAGDETECT_TORCH_MODEL`) trained on marker silhouettes runs asynchronously via LibTorch on the GPU, outputting a candidate bounding box and confidence score (`TAGDETECT_MAINCAM_TORCH_CONFIDENCE`).
   - The bounding box is tracked until the rover closes distance, allowing OpenCV to resolve the marker ID.
3. **Temporal Tracking**:
   - Bounding boxes are tracked across intermediate frames using OpenCV KCF/CSRT trackers (`src/util/vision/BoundingBoxTracking.cpp`).
   - Tags must persist for longer than `constants::BBOX_MIN_LIFETIME_THRESHOLD` (e.g., 0.5 seconds) and occupy at least `constants::BBOX_MIN_SCREEN_PERCENTAGE` of the camera frame to be considered valid targets.

### B. Object Detection (`ObjectDetector.cpp`)
Target props (mallet, rock pick, water bottle) lack distinct geometric fiducials and are detected using deep neural networks:
1. **LibTorch Inference**:
   - Custom YOLO models (`OBJECTDETECT_TORCH_MODEL`) are loaded as TorchScript (`.pt`) files via `yolomodel::pytorch::PyTorchInterpreter`.
   - Image tensors are formatted (640x640 letterboxed, RGB, normalized $[0.0, 1.0]$) and executed on CUDA.
2. **Non-Maximum Suppression (NMS)**:
   - Output tensors containing bounding boxes $[x, y, w, h]$, class IDs, and class confidences are filtered by `OBJECTDETECT_MAINCAM_TORCH_CONFIDENCE` (e.g., 0.60) and merged using `cv::dnn::NMSBoxes` with an IoU threshold (`OBJECTDETECT_MAINCAM_TORCH_NMS_THRESH`, e.g., 0.45).
3. **Multi-Object Tracking**:
   - Active detections are registered into `tracking::MultiTracker`, maintaining bounding box state during turns or temporary frame drops.

---

## 3. Geolocation Engine (`Geolocate.hpp`)

Converting 2D pixel coordinates $(u, v)$ into 3D global UTM waypoints is performed by `geoloc::GeolocateBox()`:

1. **Neighborhood Depth Sampling**:
   - Extracts a window of size $N \times N$ (default $5 \times 5$) centered at $(u, v)$ from the ZED camera's point cloud matrix `cvPointcloud` (`CV_32FC4`).
   - Invalid coordinates ($Z \le 0$, NaNs, and infinities) are discarded.
2. **Statistical Depth Isolation**:
   - Rather than computing a simple mean of all depth values (which skews toward background terrain), the Z-depth array is sorted.
   - The algorithm selects the 20th percentile depth value:
     $$\text{Target } Z = \text{Raw } Z[\lfloor 0.20 \times \text{count} \rfloor]$$
   - This isolates the front-facing surface of the object. Points within $\pm 0.5$ meters of this target depth are averaged to determine the camera-relative centroid $(X_c, Y_c, Z_c)$.
3. **Monocular Ground Plane Raycast Fallback**:
   - If stereovision fails (due to intense specular reflection or uniform texture), the algorithm triggers a geometric pinhole raycast:
     $$\theta_{\text{ray}} = \text{atan2}(v_{\text{bottom}} - c_y, f_y)$$
     $$Z_c = \frac{h_{\text{camera}}}{\tan(\theta_{\text{ray}})}$$
     $$X_c = \frac{u - c_x}{f_y} \times Z_c, \quad Y_c = -h_{\text{camera}}$$
4. **Global UTM Projection**:
   - Rover compass heading is converted to standard mathematical radians:
     $$\alpha = \left((-\theta_{\text{compass}} + 90.0) \pmod{360}\right) \times \frac{\pi}{180}$$
   - Rotates the horizontal vector $(X_c, Z_c)$ by $\alpha$ and translates by camera UTM position $(E_c, N_c)$:
     $$E_{\text{object}} = E_c + (Z_c \cos \alpha + X_c \sin \alpha)$$
     $$N_{\text{object}} = N_c + (Z_c \sin \alpha - X_c \cos \alpha)$$
     $$\text{Alt}_{\text{object}} = \text{Alt}_c + Y_c$$
5. **Radius Estimation**:
   - Computes Euclidean distance from each filtered point to the centroid, extracts the median distance, and applies an empirical scaling factor of $1.5$ to calculate the object's clearance radius.

---

## 4. Inputs, Outputs, and Limitations

### Inputs
- RGB color frames (`cv::Mat` or `cv::cuda::GpuMat`) from ZED 2i or WebRTC simulation.
- 3D spatial point cloud (`cv::Mat` formatted as `CV_32FC4`).
- Rover pose (`geoops::RoverPose`) including Easting, Northing, Altitude, and fused compass heading.

### Outputs
- `geoops::Waypoint`: Complete global target coordinate with UTM position, target classification type (`WaypointType`), and clearance radius.
- Pixel centroid, bounding box, relative distance, and yaw offset angle.

### Operational Limitations
- **Stereo Depth Degradation**: Stereoscopic depth error scales quadratically with distance. Objects beyond 15 to 20 meters produce higher geolocation uncertainty.
- **Direct Sunlight Specular Glare**: Can wash out ArUco marker contrast, requiring the rover to rely on YOLO bounding-box visual servoing until close enough for corner extraction.
- **Rotational Blur**: High angular turn rates cause pixel smearing across the CMOS sensor. The drive kinematics damp turning rates during active tracking to preserve frame sharpness.
