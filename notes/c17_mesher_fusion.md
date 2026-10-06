# c17_mesher_fusion — [0x180197E70, 0x1801B0310)

147 functions. Contents, in image order:

| range | object (guess) | contents |
|---|---|---|
| 0x180197E70–0x180198C90 | `src/plane/histogram.cpp` (tail; ctor/dtor/operator= are in the previous chunk) | Kimera-VIO `Histogram`, Pimax-modified |
| 0x180198C90–0x18019A5D0 | `src/plane/mesh.cpp` | Kimera-VIO `Mesh<Vertex2D/3D>` (no vertex colours) + STL/OpenCV helpers |
| 0x18019A5D0–0x1801A3E00 | `src/plane/mesher.cpp` | `pimax::totem::Mesher` (Kimera-VIO Mesher, heavily modified: horizontal-plane segmentation, convex hulls, ground-plane bookkeeping) + `Plane` helpers + STL |
| 0x1801A3E00–0x1801A7AD0 | `src/sensor_fusion/*` | `pimax::common::Deque<Vector3d>`, `DequeHolder` dtor, `pimax::ThreeDof::{GyroscopeBiasEstimator, LowpassFilter, MeanFilter, MedianFilter, Rotation, ImuFilter, ThreeDofTracker}` |
| 0x1801A7AD0–0x1801A8170 | `src/tracker/feature_tracker.cpp` | rpg_svo_pro `FeatureTracker` ctor / reset / resetTerminatedTracks (upstream-identical) |
| 0x1801A8170–0x1801AFEF0 | third party `fast` (uzh-rpg/fast) | FAST-10 detector/score/nonmax (upstream, generated code); SSE2 wrapper modified for MSVC |
| 0x1801AFEF0–0x1801B0310 | third party tinyxml2 | `XMLDocument` ctor + `CreateUnlinkedNode<T>` (the tinyxml2 object starts here, a bit before the 0x1801B0310 estimate) |

Reference sources used: Kimera-VIO cloned into `reference/Kimera-VIO` (full history; the
binary matches the 2019-07..2020-05 era — vertex normals present, `mesh_3d_.getPolygon`
CHECKs — with Pimax removing the vertex colours); ROS `imu_tools` cloned into
`reference/imu_tools` (imu_filter_madgwick); `uzh-rpg/fast` cloned into `reference/fast`;
`reference/rpg_svo_pro_open/svo_tracker`; `../LedObjectPoseEstimator/src/{ctrl_three_dof,common}`.

Drafts (all under `draft/c17_mesher_fusion/`):

| file | lines | content |
|---|---|---|
| plane/plane.h | 128 | `Plane`, `PolygonVertex`, `LmkPositionMap`, `LmkPixelMap`, `Plane::geometricEqual` |
| plane/histogram.h / .cpp | 84 / 140 | `Histogram` decl + `calculateHistogram`, `findPeaks`, `getLocalMaximum1D` |
| plane/mesh.h / .cpp | 110 / 149 | `Mesh<>` template + explicit instantiations |
| plane/mesher.h / .cpp | 234 / 1232 | `Mesher`, globals, free helpers `isPointInPolygon`, `polygonsOverlap`, `mergePolygons` |
| sensor_fusion/deque.h, deque_holder.h | 138 / 36 | ported from LedObjectPoseEstimator (identical code) |
| sensor_fusion/vector.h, rotation.h | 122 / 68 | ported (identical) |
| sensor_fusion/filters.h / .cpp | 69 / 114 | ported (identical) |
| sensor_fusion/gyroscope_bias_estimator.h / .cpp | 114 / 211 | ported, threshold 1000.0f |
| sensor_fusion/imu_filter.h / .cpp | 81 / 161 | float ROS Madgwick + Pimax world frame 3 |
| sensor_fusion/three_dof_tracker.h / .cpp | 106 / 165 | Pimax 3-DoF tracker |
| tracker/feature_tracker.h / .cpp | 62 / 45 | svo FeatureTracker (ctor, reset, resetTerminatedTracks) |
| third_party/fast/faster_corner_10_sse.cpp | 34 | modified `fast_corner_detect_10_sse2` wrapper |

(3603 lines total.)

---------------------------------------------------------------------------------------------

## 1. Function table

kind `project` = reconstructed in the draft. Upstream status relative to Kimera-VIO
(plane/*), ROS imu_filter_madgwick (ImuFilter), LedObjectPoseEstimator (other sensor_fusion
classes, which are themselves Cardboard-SDK ports), rpg_svo_pro (FeatureTracker).

### 1.1 Histogram (src/plane/histogram.cpp)

| address | name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x180197E70 | `pimax::totem::Histogram::calculateHistogram` | `void (const cv::Mat& input, bool log_histogram)` | project | modified: non-static range arrays, no LOG(FATAL) for dims∉{1,2}, dims_ is 64-bit | cv::calcHist for 1/2-D; optional FileStorage dump `histogram_<dims>.yaml` |
| 0x180198460 | `Histogram::findPeaks` | `std::vector<PeakInfo> (cv::InputArray, int window_size) const` | project | modified: slope = src(i) − src(i−size) (Kimera src(i+size) − src(i−size)); up-hill end also on 0→1; CHECK removed; peakInfo inlined | slope state machine, peaks with left/right hill sizes |
| 0x180198960 | `Histogram::getLocalMaximum1D` | `std::vector<PeakInfo> (const cv::Size&, int window, float peak_per, float min_support, float reference_value, bool is_first_frame) const` | project | modified: new acceptance rule & params, no CHECK/log/draw | Gaussian blur, findPeaks, keep peaks > peak_per·max and (> min_support or (> 10 and (first frame or \|value−ref\|>1))) with hill sizes ≥ 2 |

### 1.2 Mesh (src/plane/mesh.cpp)

| address | name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x180198C90 | `std::_Tree<map<int,int>>::_Find_hint` | | lib:std::map insert-hint search | - | used by 0x180199300 |
| 0x180198E90 | `std::vector<Mesh3D::VertexType>::_Resize_reallocate` | | lib:std::vector (28-byte elem, default -1 id) | - | `polygon.resize(n)` growth |
| 0x1801990C0 | `std::vector<Mesh2D::VertexType>::_Resize_reallocate` | | lib:std::vector (24-byte elem) | - | |
| 0x180199300 | `std::_Tree<map<int,int>>::_Insert_range_unchecked` | | lib:std::map range insert | - | used by map copy-assign in Mesh::operator= |
| 0x180199450 | `cv::Mat::push_back<int>` | | lib:OpenCV inline | - | |
| 0x180199580 | `cv::Mat::push_back<cv::Point3f>` | | lib:OpenCV inline | - | |
| 0x1801996D0 | `Mesh<VertexPosition>::Mesh(const size_t& polygon_dimension)` | | project | modified: CHECK_GE removed, no colour Mat (2D/3D COMDAT-folded) | also called by FrameProcessorBase 0x1800F2070 (Mesh2D) |
| 0x1801997A0 | `Mesh3D::operator=(const Mesh3D&)` | | project | modified: CHECK_EQ → `std::cout << "Error: The Mesh that you are trying to copy has different dimensions for the polygons!" << std::endl` | deep copy |
| 0x180199900 | `std::map<int,int>::operator[]` | | lib:std::map try_emplace | - | |
| 0x180199A00 | `Mesh3D::addPolygonToMesh` | `void (const Polygon&)` | project | modified: CHECK_EQ → if; updateMeshDataStructures inlined | |
| 0x180199C30 | `Mesh2D::addPolygonToMesh` | | project | modified (same) | |
| 0x180199D30 | `Mesh<>::clearMesh` | `void ()` | project | modified: no colour Mat, maps cleared lmk→vtx first | 2D/3D folded |
| 0x180199E10 | `cv::Mat_<cv::Point3f>::clone` (Mat_(const Mat&) conversion) | | lib:OpenCV inline | - | |
| 0x180199F40 | `Mesh3D::getPolygon` | `bool (const size_t&, Polygon*) const` | project | modified: CHECK_NOTNULL/VLOG removed, the 2 DCHECKs became `return false` | |
| 0x18019A300 | `Mesh2D::updateMeshDataStructures` | `void (const LandmarkId&, const Point2f&, map*, map*, Mat*, Mat_<Point3f>*, Mat*) const` | project | modified: CHECK_NOTNULLs + DCHECK(!normals_computed_) → one guard; no colour | |

### 1.3 Mesher (src/plane/mesher.cpp)

| address | name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x18019A5D0 | `std::list<pair<const int,Vector3f>>::_Assign_cast` | | lib:std::unordered_map copy-assign helper | - | for Plane::lmk_ids_map_ |
| 0x18019A7B0 | `std::vector<PolygonVertex>::_Assign_range` | | lib:std::vector | - | |
| 0x18019A990 | `std::vector<Plane>::_Assign_range` | | lib:std::vector operator= | - | `*non_associated_planes = segmented_planes` |
| 0x18019ABD0 | `std::vector<cv::Point3f>::_Assign_range` | | lib:std::vector | - | Plane::unknown_24_ |
| 0x18019ADC0 | `std::_Copy_unchecked<Plane*>` | | lib: implicit `Plane::operator=` loop | - | reveals the full Plane layout |
| 0x18019AF80 | `vector<Vector3f>::_Emplace_reallocate<(p/z)*s expr>` | | lib:std::vector + Eigen | - | points_rescaled.emplace_back |
| 0x18019B150 | `vector<Vector3f>::_Emplace_reallocate<Rᵀ(p−t) expr>` | | lib:std::vector + Eigen | - | points.emplace_back |
| 0x18019B2E0 | `vector<uint64_t>::_Emplace_reallocate<int&>` | | lib:std::vector | - | |
| 0x18019B3E0 | `vector<Plane>::_Emplace_reallocate<const Plane&>` | | lib:std::vector | - | |
| 0x18019B540 | `vector<Mesh3D::Polygon>::_Emplace_reallocate<const Polygon&>` | | lib:std::vector | - | wall_polygons |
| 0x18019B800 | `vector<Plane>::_Emplace_reallocate<Plane>` | | lib:std::vector (move) | - | |
| 0x18019B960 | `vector<PolygonVertex>::_Insert_range` | | lib:std::vector insert(pos,first,last) | - | |
| 0x18019BBD0 | `std::_Uninitialized_move<PolygonVertex*>` | | lib (EH funclet helper) | - | |
| 0x18019BC10 | `std::_Uninitialized_copy<PolygonVertex*>` | | lib | - | |
| 0x18019BC60 | construct `Eigen::Vector3f` from `Rᵀ*(p−t)` in place | | lib:Eigen lazy product coeffs | - | |
| 0x18019BE00 | `std::vector<Mesh3D::VertexType>` copy ctor | | lib:std::vector | - | Polygon copy |
| 0x18019BEF0 | `cv::Mat::push_back<float>` | | lib:OpenCV inline | - | |
| 0x18019C020 | `pimax::totem::Mesher::Mesher()` | | project | modified: no MesherParams/hist_2d_ (args still built), z hist 200 bins [-4,4] | via make_shared in FrameProcessorBase ctor 0x1800DFDF0 |
| 0x18019C640 | `Plane::Plane(Plane&&)` | | project (compiler-generated) | - | |
| 0x18019C8A0 | list-node free helper (unordered_map member unwind) | | lib:EH funclet helper (Mesher ctor) | - | |
| 0x18019C8F0 | `std::_Destroy_range<Plane*>` | | lib (EH) | - | |
| 0x18019C930 | `std::vector<Mesh3D::Polygon>::_Tidy` | | lib | - | |
| 0x18019C9F0 | `_Clear_guard` (unordered_map ctor unwind: `if (p) p->clear()`) | | lib (EH) | - | |
| 0x18019CA00 | `pimax::totem::mergePolygons` | `void (vector<PolygonVertex>* a, const vector<PolygonVertex>& b, const Vector3f& normal_a, bool* clockwise)` | project | new | copies b into empty a; otherwise computes & discards a convex hull (QUIRK) |
| 0x18019D000 | `Mesher::populate3dMesh` | `bool (const vector<Vec6f>&, const LandmarkIds& triangle_lmk_ids, const LmkPositionMap&, double, double, double, Mesh2D*)` | project | modified heavily (see §5) | builds mesh_3d from 2D triangles |
| 0x18019DB90 | `Mesher::populate3dMeshTimeHorizon` | same params | project | modified: returns bool | populate + updatePolygonMeshToTimeHorizon |
| 0x18019DC30 | `Mesher::getRatioBetweenTangentialAndRadialDisplacement` | `double (p1,p2,p3, const Matrix3f& R_w_c, const Vector3f& t_w_c) const` | project | modified: float Eigen instead of gtsam, UtilsGeometry inlined, no LOG | |
| 0x18019E1A0 | `Mesher::isBadTriangle` | `bool (const Polygon&, R, t, const double&×3) const` | project | modified: size check → bad; early-return order | |
| 0x18019E480 | `pimax::totem::polygonsOverlap` | `bool (const vector<PolygonVertex>&, vector<int>, const vector<PolygonVertex>&, vector<int>, const LmkPositionMap&)` | project | new | ≥ g_plane_overlap_min_points vertices/landmarks inside the other hull |
| 0x18019EF20 | `Mesher::isPolygonInPlaneHull` | `bool (const Polygon&, const Plane&) const` | project | new | any triangle vertex inside plane hull (plane frame) |
| 0x18019F330 | `pimax::totem::isPointInPolygon` | `bool (float x, float y, const vector<Point2f>&)` | project | new (PNPOLY) | empty polygon ⇒ true |
| 0x18019F590 | `Mesher::updatePolygonMeshToTimeHorizon` | `void (const LmkPositionMap&)` | project | modified: always reduce, no re-filter, member mesh_output | |
| 0x18019F7A0 | `std::vector<Plane>::_Change_array` | | lib | - | |
| 0x18019F880 | `std::_Destroy_range<Plane*>` | | lib (EH) | - | |
| 0x18019F8C0 | `std::_Destroy_range<Mesh3D::Polygon*>` | | lib (EH) | - | |
| 0x18019F900 | `std::_Uninitialized_move<Plane*>` | | lib | - | |
| 0x18019F970 | `std::_Uninitialized_move<PolygonVertex*>` | | lib | - | |
| 0x18019F9B0 | `std::_Uninitialized_copy<Plane*>` | | lib | - | |
| 0x18019FA20 | `std::allocator<Plane>::allocate` | | lib | - | |
| 0x18019FA90 | `Mesher::associatePlanes` | `void (const vector<Plane>& seg, const vector<Plane>& planes, vector<Plane>* non_assoc, vector<Plane>* assoc_seg, LandmarkIds* assoc_ids, const LmkPositionMap&, const double& ntol, const double& dtol) const` | project | modified heavily | |
| 0x1801A0870 | `Mesher::calculateNormal` | `bool (p1,p2,p3, cv::Point3f*) const` | project | modified: CHECKs/LOG removed, final CHECK_GT → return false | |
| 0x1801A0AF0 | `Mesher::clusterPlanesFromMesh` | `void (vector<Plane>* planes, vector<Plane>* new_planes, const LmkPositionMap&)` | project | modified heavily | main entry from FrameProcessorBase 0x1800F2070 |
| 0x1801A1380 | `std::allocator<Plane>::deallocate` | | lib | - | |
| 0x1801A13D0 | `Plane::geometricEqual` | `bool (const Plane& rhs, const double& ntol, const double& dtol) const` | project | modified: CHECKs removed (Kimera isNormalStrictlyEqual / isPlaneDistanceStrictlyEqual inlined) | out-of-line copy of an inline header function |
| 0x1801A1500 | `Mesher::updatePlanesPolygon` | `void (vector<Plane>*, const LmkPositionMap&) const` | project | new | recompute every plane's convex hull |
| 0x1801A1DB0 | `Mesher::segmentHorizontalPlanes` | `void (vector<Plane>*, size_t* plane_id, const Plane::Normal&, const cv::Mat& z)` | project | modified | z histogram peaks → horizontal planes |
| 0x1801A24D0 | `Mesher::segmentPlanesInMesh` (+ inlined segmentNewPlanes, getLongitude, isNormal*) | `void (vector<Plane>* seed, vector<Plane>* new, const LmkPositionMap&, const double&×4)` | project | modified | |
| 0x1801A3310 | `Mesher::updatePlanesLmkIdsFromMesh` | `void (vector<Plane>*, double ntol, double dtol, const LmkPositionMap&) const` | project | modified | |
| 0x1801A3750 | `Mesher::updatePlanesLmkIdsFromPolygon` | `bool (vector<Plane>*, const Polygon&, const size_t&, const Point3f&, double ntol, double dtol, double scale, const LmkPositionMap&, bool) const` | project | modified heavily | |

### 1.4 sensor_fusion

| address | name | signature | kind | upstream (vs LedObjectPoseEstimator) | summary |
|---|---|---|---|---|---|
| 0x1801A3E00 | `pimax::DequeHolder::~DequeHolder` | | project (compiler-generated) | identical (Led 0x180106640) | string +176, Deques +216/+272/+424, vector<Vector3d> +400 |
| 0x1801A3ED0 | `pimax::common::Deque<Vector3d>::scalar deleting dtor` | | project | identical | vtable 0x1803BF310 slot 0 |
| 0x1801A3F20 | `Deque::clear` | | project | identical | slot 11 (returns 0) |
| 0x1801A3F30 | `Deque::tail` | | project | identical | slot 14; COMDAT-folded with Ceres getters |
| 0x1801A3F40 | `Deque::capacity` | | project | identical | slot 10; folded |
| 0x1801A3F50 | `Deque::size` | | project | identical | slot 9; folded |
| 0x1801A3F60 | `Deque::empty` | | project | identical | slot 12 |
| 0x1801A3F70 | `Deque::full` | | project | identical | slot 13 |
| 0x1801A3F80 | `Deque::from_back` | | project | identical (assert line 0xB2) | slot 5 |
| 0x1801A3FE0 | `Deque::from_back const` | | project | identical (0xC7) | slot 6 |
| 0x1801A4040 | `Deque::operator[]` | | project | identical (0xA8) | slot 7 |
| 0x1801A40A0 | `Deque::operator[] const` | | project | identical (0xBD) | slot 8 |
| 0x1801A4100 | `Deque::pop_back` | | project | identical (0x98) | slot 3 |
| 0x1801A4180 | `Deque::pop_front` | | project | identical (0x8A) | slot 4 |
| 0x1801A4200 | `Deque::push_back` | | project | identical (0x6F) | slot 1 |
| 0x1801A4280 | `Deque::push_front` | | project | identical (0x7C) | slot 2 |
| 0x1801A42F0 | jmp thunk → 0x180172B70 (`std::vector<24-byte T>::_Tidy`) | | lib (thunk; called from Ceres 0x1802152F0) | - | |
| 0x1801A4300 | `pimax::ThreeDof::GyroscopeBiasEstimator::GyroscopeBiasEstimator` | | project | identical | |
| 0x1801A4450 | `std::deque<Vector3>::~deque` | | lib | - | MeanFilter/MedianFilter member |
| 0x1801A4540 | `GyroscopeBiasEstimator::~GyroscopeBiasEstimator` | | project (compiler-generated) | identical | |
| 0x1801A45B0 | `MeanFilter::~MeanFilter` | | project (compiler-generated) | identical | |
| 0x1801A45C0 | `MedianFilter::~MedianFilter` | | project (compiler-generated) | identical | |
| 0x1801A4690 | `GyroscopeBiasEstimator::scalar deleting dtor` | | project | identical | vtable 0x1803BF4A8 slot 0 |
| 0x1801A4710 | `GyroscopeBiasEstimator::GetGyroscopeBias(double&,double&,double&) const` | | project | identical (out of line here) | |
| 0x1801A4760 | `GetGyroscopeBias() const` | `Vector3 ()` | project | identical | slot 5 |
| 0x1801A4780 | `IsCurrentEstimateValid() const` | `bool ()` | project | modified: weight threshold 1000.0f (Led 250, Cardboard 25) | slot 6 |
| 0x1801A4920 | `ProcessAccelerometer(const double&×3, uint64_t)` | | project | identical | slot 4 |
| 0x1801A4980 | `ProcessAccelerometer(const Vector3&, uint64_t)` | | project | identical | slot 3 |
| 0x1801A4C60 | `ProcessGyroscope(const double&×3, uint64_t)` | | project | identical | slot 2 |
| 0x1801A4CC0 | `ProcessGyroscope(const Vector3&, uint64_t)` | | project | identical | slot 1 |
| 0x1801A4E60 | `GyroscopeBiasEstimator::Reset` | | project | identical (out of line here) | |
| 0x1801A4EB0 | `LowpassFilter::LowpassFilter(double)` | | project | identical | |
| 0x1801A4EE0 | `LowpassFilter::AddSample` | | project | identical | |
| 0x1801A4FC0 | `LowpassFilter::AddWeightedSample` | | project | identical | |
| 0x1801A50A0 | `LowpassFilter::Reset` | | project | identical | |
| 0x1801A50C0 | `MeanFilter::MeanFilter(size_t)` | | project | identical | |
| 0x1801A5120 | `MeanFilter::AddSample` | | project | identical | |
| 0x1801A51F0 | `MeanFilter::GetFilteredData` | | project | identical | |
| 0x1801A52D0 | `std::deque<Vector3>::_Growmap` | | lib | - | |
| 0x1801A54B0 | `MedianFilter::MedianFilter(size_t)` | | project | identical | |
| 0x1801A5540 | `MedianFilter::AddSample` | | project | identical | |
| 0x1801A56E0 | `MedianFilter::GetFilteredData` | | project | identical (no-zero-fallback quirk) | |
| 0x1801A5A00 | `MedianFilter::IsValid` | | project | identical | |
| 0x1801A5A10 | `Rotation::GetAxisAndAngle` | | project | identical | |
| 0x1801A5B20 | `Rotation::RotateInto` | | project | identical | |
| 0x1801A5D10 | `Cross(Vector<3>, Vector<3>)` | | project | identical | |
| 0x1801A5D70 | `Dot<3>` | | project | identical | |
| 0x1801A5DA0 | `Dot<4>` | | project | identical | |
| 0x1801A5DE0 | `pimax::ThreeDof::ImuFilter::ImuFilter` | | project | modified vs ROS: gain 0.05f, zeta 0.001f, NWU (Led: double, 0.05/0.001, frame unused) | |
| 0x1801A5E20 | `ImuFilter::scalar deleting dtor` | | project | identical | vtable 0x1803BF528 |
| 0x1801A5E50 | `addGradientDescentStep` (static) | | project | ROS-identical (float) | |
| 0x1801A61B0 | `ImuFilter::madgwickAHRSupdateIMU` | `void (float gx,gy,gz,ax,ay,az,dt)` | project | ROS float version + world frame 3 (gravity +X) | Led version: double, +Y only |
| 0x1801A6630 | `rotateAndScaleVector` (static) | | project | ROS-identical (float) | |
| 0x1801A67D0 | `std::vector<ImuSample>::_Insert_range` | | lib | - | init_buffer_.insert(end, …) |
| 0x1801A6A70 | `std::_Move_unchecked<ImuSample*>` | | lib | - | erase |
| 0x1801A6AC0 | `pimax::ThreeDof::ThreeDofTracker::ThreeDofTracker` | | project | new | |
| 0x1801A6C10 | `ThreeDof::State::operator=` | | project (compiler-generated) | - | 72 bytes copied |
| 0x1801A6C70 | `_Ref_count_obj2<ImuFilter>::_Delete_this` (COMDAT-folded, named `??_G?$codecvt…`) | | lib | - | |
| 0x1801A6CA0 | `ThreeDofTracker::PropagateGyroOnly` | `void (const ImuSample&, const vector<ImuSample>&, const State&, State*)` | project | new | |
| 0x1801A7030 | `ThreeDofTracker::Initialize` | `bool (const vector<ImuSample>&)` | project | new | gyro bias from static window |
| 0x1801A7380 | `ThreeDofTracker::InitState` | `void (const vector<ImuSample>&, State*)` | project | new | |
| 0x1801A7540 | `ThreeDofTracker::Propagate` | `void (const ImuSample& last, const vector<ImuSample>&, const State& in, State* out)` | project | new | c14 name |
| 0x1801A7A50 | `ThreeDofTracker::SetBias` | `void (const Vector3f& bg, const Vector3f& ba)` | project | new | c14 calls it SetImuExtrinsics |
| 0x1801A7A80 | `std::_Uninitialized_move<ImuSample*>` | | lib | - | |

### 1.5 tracker / fast / tinyxml2

| address | name | signature | kind | upstream | summary |
|---|---|---|---|---|---|
| 0x1801A7AD0 | `vector<shared_ptr<AbstractDetector>>::_Emplace_reallocate` | | lib | - | |
| 0x1801A7D00 | `vector<FeatureTracks, aligned_allocator>::vector(size_t)` | | lib (Eigen aligned alloc) | - | |
| 0x1801A7DF0 | `FeatureTracker::FeatureTracker(const FeatureTrackerOptions&, const DetectorOptions&, const CameraBundlePtr&)` | | project | identical | from 0x18012B320 (AbstractInitialization) |
| 0x1801A8030 | `FeatureTracker::reset` | | project | identical | from AbstractInitialization vtable fn 0x18012C030 |
| 0x1801A80F0 | `FeatureTracker::resetTerminatedTracks` | | project | identical | |
| 0x1801A8170 | `fast::fast_corner_detect_10` | | lib:fast (generated) | identical | spot-checked prologue/offsets |
| 0x1801AB160 | `fast::fast_corner_score_10(const fast_byte*, const int[], int)` | | lib:fast (generated, static) | identical | |
| 0x1801AF880 | `fast::fast_corner_score_10(img, stride, corners, threshold, scores)` | | lib:fast | identical | |
| 0x1801AFA30 | `fast::fast_corner_detect_10_sse2` | | lib:fast | modified (MSVC: no SSE2 path) | drafted (third_party/fast) |
| 0x1801AFA50 | `fast::fast_nonmax_3x3` | | lib:fast | identical | |
| 0x1801AFEF0 | `tinyxml2::XMLDocument::CreateUnlinkedNode<XMLComment>` | | lib:tinyxml2 | - | |
| 0x1801AFF70 | `…CreateUnlinkedNode<XMLDeclaration>` | | lib:tinyxml2 | - | |
| 0x1801AFFF0 | `…CreateUnlinkedNode<XMLElement>` | | lib:tinyxml2 | - | |
| 0x1801B0080 | `…CreateUnlinkedNode<XMLText>` | | lib:tinyxml2 | - | |
| 0x1801B0110 | `…CreateUnlinkedNode<XMLUnknown>` | | lib:tinyxml2 | - | |
| 0x1801B0190 | `tinyxml2::XMLDocument::XMLDocument(bool processEntities, Whitespace)` | | lib:tinyxml2 | - | from the calibration parser 0x18015E250 |

---------------------------------------------------------------------------------------------

## 2. Types

### 2.1 `pimax::totem::Histogram` (sizeof 0x1B0 = 432; temp of 432 bytes in Mesher ctor)

| off | type | name | evidence |
|---|---|---|---|
| 0x00 | int | n_images_ | calcHist nimages (sure) |
| 0x08 | int* | channels_ | calcHist channels (sure) |
| 0x10 | cv::Mat | mask_ | calcHist mask (sure) |
| 0x70 | size_t (64-bit) | dims_ | qword compare with 1/2, unsigned to_string (sure it is 8 bytes) |
| 0x78 | int* | hist_size_ | (sure) |
| 0x80 | float** | ranges_ | (sure) |
| 0x88 | bool | uniform_ | (sure) |
| 0x89 | bool | accumulate_ | (sure) |
| 0x90 | cv::Mat | histogram_ | (sure) |
| 0xF0 | cv::Mat | unknown_f0_ | default ctor/dtor only |
| 0x150 | cv::Mat | unknown_150_ | default ctor/dtor only |

`PeakInfo` (24 bytes): pos_ +0, left_size_ +4, right_size_ +8, float support_ +12, double value_ +16.
Kimera layout + 2 extra Mats; no hist_2d_ in Mesher.

### 2.2 `Mesh<VertexPosition>` (sizeof 0x150 = 336)

| off | type | name |
|---|---|---|
| 0x00 | std::map<int,int> | vertex_to_lmk_id_map_ |
| 0x10 | std::map<int,int> | lmk_id_to_vertex_map_ |
| 0x20 | cv::Mat | vertices_mesh_ (CV_32FC3 after ctor/clear, even for 2D) |
| 0x80 | cv::Mat_<Point3f> | vertices_mesh_normal_ |
| 0xE0 | bool | normals_computed_ |
| 0xE8 | cv::Mat | polygons_mesh_ (CV_32SC1) |
| 0x148 | const size_t | polygon_dimension_ |

All offsets sure (getPolygon/addPolygon/operator=/clearMesh). Kimera (v5) has an extra
`vertices_mesh_color_` Mat and a colour in Vertex — both removed.
`Vertex<Point3f>` 28 bytes {int lmk_id_ (-1); Point3f position; Point3f normal};
`Vertex<Point2f>` 24 bytes.

### 2.3 `Plane` (sizeof 232) — see `plane/plane.h` for the table (+0 id, +4 normal_ Point3f,
+16 distance_, +24 vector<Point3f>, +48 unordered_map<int,12-byte>, +112 Vector3f centroid_,
+128 lmk_ids_, +152 vector<PolygonVertex> polygon_, +176 cluster_id_, +184 all_lmk_ids_,
+208 is_valid_, +209 hull_clockwise_, +210 needs_wall_refit_, +212 age_, +216 distance_ref_,
+224 area_). Proven here: +112 is a float centroid (accumulated / 3.0f, Eigen norm), +48 is an
unordered_map whose node holds {int key, 12-byte value} (0x18019A5D0 list assign), the full
member list from the copy-assign loop 0x18019ADC0, the inline ctor (segmentHorizontalPlanes):
+208/+209/+210 = true, +212 = 0, +216 = +224 = 0, all containers cleared.
c08 (`draft/c08_preint_ground/plane/plane_types.h`) names reused.

`PolygonVertex` 16 bytes {int lmk_id; Vector3f pos}. `LmkPositionMap` = unordered_map<int,
Vector3f> (node {key +16, value +20}). `LmkPixelMap` = unordered_map<int, Vector2d?> (node
0x30, value compared at +32/+40 with pixel x/y as doubles) — value type per c08.

### 2.4 `pimax::totem::Mesher` (sizeof 0x4C8 = 1224; make_shared block 0x4D8)

| off | type | name | evidence |
|---|---|---|---|
| 0x000 | Mesh3D | mesh_3d | CHECK text "mesh_3d.getPolygon(i, &polygon)" (sure) |
| 0x150 | Mesh3D | mesh_output | updatePolygonMeshToTimeHorizon (sure) |
| 0x2A0 | Histogram | z_hist_ | ctor / segmentHorizontalPlanes (sure) |
| 0x450 | Eigen::Matrix3f | R_w_c_ | identity in ctor; written by 0x1800F2070; R(2,2) at 0x470 (sure) |
| 0x474 | Eigen::Vector3f | t_w_c_ | camera position (sure) |
| 0x480 | LmkPixelMap | lmk_px_ | iterated in populate3dMesh (sure) |
| 0x4C0 | bool | is_first_frame_ | = planes->empty() (sure) |

No vtable. Kimera members `hist_2d_`, `mesher_params_` gone.

### 2.5 sensor_fusion

* `pimax::common::Deque<Eigen::Vector3d>` — 32 bytes, vtable 0x1803BF310 (15 slots, see table).
  Same as Led.
* `pimax::DequeHolder` (name unknown) — dtor only (0x1801A3E00); SLAMManager+2840.
* `pimax::ThreeDof::GyroscopeBiasEstimator` — 384 bytes, vtable 0x1803BF4A8 (7 slots); layout
  identical to Led (see header comment).
* `LowpassFilter` 48, `MeanFilter` 48, `MedianFilter` 88 bytes — identical to Led.
* `pimax::ThreeDof::ImuFilter` — 48 bytes, vtable 0x1803BF528 [dtor]; float gain_ +8, zeta_ +12,
  int world_frame_ +16, q0..q3 +20..+32, w_b* +36..+44. make_shared control block
  `_Ref_count_obj2<ImuFilter>` vtable 0x1803BF548 (0x40 bytes).
* `pimax::ThreeDof::ThreeDofTracker` (name TODO(verify); no RTTI) — 0x2A0 bytes:

| off | type | name | evidence |
|---|---|---|---|
| 0 | shared_ptr<ImuFilter> | filter_ | ctor (sure) |
| 16 | std::mutex | mutex_ | `_Mtx_init_in_situ(+16, 2)` (sure; never locked here) |
| 96 | bool | ready_ | Propagate (sure; c14 name) |
| 97 | bool | reset_ | forces Initialize (name guess) |
| 98 | bool | use_accelerometer_ | latched in Initialize (name guess) |
| 100 | Vector3f | bg_ | subtracted from gyro (sure) |
| 112 | Vector3f | ba_ | only stored (sure it is SetBias' 2nd arg) |
| 128 | State | state_ | (sure) |
| 208 | ImuSample | last_imu_ | (sure; c14 "config_") |
| 240 | vector<ImuSample> | init_buffer_ | (sure) |
| 264 | 8 bytes | unknown_264_ = 0 | ctor only |
| 272 | float | gravity_ = 9.80667f | ctor only |
| 280 | GyroscopeBiasEstimator | bias_estimator_ | (sure) |

* `ThreeDof::State` — sizeof 80 (alignment 16): p +0, v +12, ba +24, bg +36, Quaternionf q +48,
  double t +64. **c14 asserts 72 — wrong.**
* `ThreeDof::ImuSample` — 32 bytes: acc +0, gyr +12, double t +24. **c14 declares
  {t, acc, gyr} — wrong order** (Propagate reads gyr at +12..+20, acc at +0..+8, t at +24;
  Initialize reads back().t at end−8).

### 2.6 `FeatureTracker` (152 bytes) — upstream svo layout: options_ +0 (72 bytes), bundle_size_
+72, detectors_ +80, active_tracks_ +104, terminated_tracks_ +128. FeatureTrackerOptions copied
member-wise exactly as upstream (+0 int, +4 int, +8 vector<int>, +32 int, +40 double, +48 bool,
+56 size_t, +64 bool).

---------------------------------------------------------------------------------------------

## 3. External interfaces

### 3.1 Calls into the chunk from outside

| caller | callee | meaning |
|---|---|---|
| 0x1800DFDF0 FrameProcessorBase ctor | 0x18019C020 | `std::make_shared<Mesher>()` (FrameProcessorBase+3592) |
| 0x1800F2070 FrameProcessorBase (mesh update) | 0x1801996D0, 0x18019DB90, 0x1801A0AF0 | Mesh2D(3); `populate3dMeshTimeHorizon(tris, lmk_ids, lmk_positions, g_240, g_248, g_250*0.5, &mesh_2d)`; `clusterPlanesFromMesh(fp+3672, fp+3696, lmk_positions)`; also writes mesher+1104 (R), +1140 (t), +1152 (lmk_px_) |
| 0x1800EC2B0 | 0x18019E480, 0x18019CA00 | polygonsOverlap, mergePolygons |
| 0x1801663F0 SLAMManager init | 0x1801A6AC0, 0x1801A4540, 0x1801A7A50, 0x1801A3E00 | new ThreeDofTracker; (inlined dtor uses ~GyroscopeBiasEstimator); SetBias(fp+536, fp+548); ~DequeHolder |
| 0x1801671D0 / 0x180167320 / 0x180167AF0 | 0x1801A3E00, 0x1801A4540 | unique_ptr deleters / ~SLAMManager |
| 0x18016B210 SLAMManager::UpdateThreeDof | 0x1801A7540 | Propagate |
| 0x18012B320 / 0x18012C030 (AbstractInitialization) | 0x1801A7DF0 / 0x1801A8030 | FeatureTracker ctor / reset |
| 0x1800A0750, 0x1800AC5E0 (feature detection) | 0x1801AFA30, 0x1801AF880, 0x1801AFA50 | FAST |
| 0x18015E250 (calibration XML) | 0x1801B0190 | tinyxml2::XMLDocument ctor |
| 0x1802152F0 (Ceres) | 0x1801A42F0 | thunk |
| 0x180247580, 0x1802488C0 (Ceres) | 0x1801A3F30/0x1801A3F40 | COMDAT-folded getters |

### 3.2 Calls out of the chunk

Histogram (previous chunk): 0x180197940 `Histogram(int, const vector<int>&, Mat, int, const vector<int>&, const vector<array<float,2>>&, bool, bool)`, 0x180197BB0 `Histogram()`, 0x180197C20 `~Histogram`, 0x180197CC0 `Histogram::operator=`, 0x1801976B0 `vector<PeakInfo>::_Emplace_reallocate`.

Plane / containers (other chunks, mostly frontend objects): 0x1800E2910 `Plane(const Plane&)`, 0x1800E5230 `~Plane`, 0x1800BDDF0 `std::_Move_unchecked<Plane>` (erase), 0x1800FEB10 `vector<Plane>::clear`, 0x1800DFA50 `vector<int>(const vector<int>&)`, 0x1800DFC00 `vector<PolygonVertex>(const&)`, 0x1800E5D00 `vector<int>::assign`, 0x1800D0460 (unordered_map range insert, from Plane copy ctor), 0x1800DF950 `unordered_map<int,Vector3f>()`, 0x1800FE8F0 `unordered_map::clear`, 0x180077970 `~unordered_map`, 0x1800B7B50 `hash<int>`, 0x1800BD000 `_Hash::_Find_last`, 0x1800CC9B0 `unordered_map::insert(value_type&&)`, 0x18011B240 `unordered_map::reserve`, 0x1800BD0C0/0x1800FA130/0x1800FA300/0x180007450/0x18000F840/0x1800F8580/0x18009A2B0 (unordered_map internals), 0x1800BB200 `vector<PolygonVertex>::_Emplace_reallocate`, 0x1800BC020 `vector<Point2f>::_Emplace_reallocate`, 0x180062DE0 `vector<int>::_Emplace_reallocate`, 0x180009820 `~vector<int>`, 0x180009880 `~vector<PolygonVertex>`, 0x180019E70 `~vector<Point2f>`, 0x18001CB30/0x18001CB80/0x1800B2D60 deallocate helpers, 0x180025640/0x1800F9550 `_Reallocate_exactly`, 0x1800FDA30/0x180011500/0x180011490/0x1800B2CF0/0x18001CA40 allocate helpers, 0x1800F79D0/0x1800F7880/0x180185490 `_Change_array`, 0x18016B700 uninitialized move ImuSample, 0x1800B2BC0 `~vector<Mesh3D::VertexType>`, 0x1800E4150 `~vector<Vector3d>`, 0x180090AD0 `std::_Partition_by_median_guess_unchecked<float*>`, 0x18012D0D0 `deque<float>::_Growmap`, 0x180012AB0 `std::map<int,int>::clear`, 0x18000FE80 `_Tree::_Insert_node`, 0x1800248E0 `_Tree iterator --`, 0x1800BEF30 `vector<int>::_Resize_reallocate`.

Eigen (other chunks): 0x1800DD270 `Quaternionf::FromTwoVectors(a, b)` (setFromTwoVectors), 0x1801284D0 `QuaternionBase<Quaternionf>::toRotationMatrix`, 0x18016D100 `Quaternionf::normalized`, 0x18016EB90 `Quaternionf operator*`, 0x180008720 / 0x1800087D0 Block<…,3,1>/<…,1,3> ctor asserts. 0x1801662D0 `ThreeDof::State::State()`.

glog: 0x180354EB0 `LogMessageFatal(file, line, const CheckOpString&)`, 0x180354E80 `LogMessageFatal(file, line)`, 0x180355280 `~LogMessageFatal`, 0x18035AC70 `LogMessage::stream`, 0x180354DC0 `CheckOpMessageBuilder(const char*)`, 0x180355D70 `ForVar2`, 0x180357BF0 `NewString`, 0x180355010 `~CheckOpMessageBuilder`.

misc: 0x180016B90 `printf`, 0x180006290 `operator<<(ostream&, const char*)`, 0x180018500 `std::endl`, 0x1800ABA70 `std::string(const char*)`, 0x180011570/0x180006C30/0x180159410 std::string assign/append/insert, 0x1800101C0/0x1800101E0/0x1800103D0/0x180010390/0x180010410 `_Xlength`/`cancel_current_task`.

FeatureTracker: 0x1801B41A0 `NCamera::getCameraShared`, 0x1800AD840 `feature_detection_utils::makeDetector`, 0x1800AAB80 `OccupandyGrid2D::reset`, 0x1800098E0 `~vector<FeatureRef>` (FeatureTrack element dtor), 0x180027A70 allocate int, 0x180006820/0x180010320/0x18000FA40/0x180012CA0 shared_ptr range helpers.

tinyxml2: 0x1801B3150 `DynArray<XMLNode*,10>::Push` (`_unlinked`).

OpenCV imports: calcHist, FileStorage ctor/dtor, `operator<<(FileStorage&, const String&)`, `write(FileStorage&, const String&, const Mat&)`, `error`, Mat ctors/dtor/clone/reshape/push_back_/convertTo/release/empty/operator=, `_InputArray::kind/getMat_`, GaussianBlur, minMaxLoc, noArray, `norm(InputArray, int, InputArray)`, convexHull(InputArray, OutputArray, bool, bool). CRT: sqrt, sqrtf, atan2f, cos, sin, acos, memmove, memset, operator new/free.

### 3.3 Globals

| address | type | proposed name | init | used by |
|---|---|---|---|---|
| 0x18046A220 | cv::Point3f | `segmentPlanesInMesh::vertical` (function static) | (0,0,1) | 0x1801A24D0 |
| 0x18046A230 | cv::Point3f | `segmentNewPlanes::vertical` (function static) | (0,0,1) | 0x1801A24D0 → 0x1801A1DB0 |
| 0x18046A240 | double | g_min_ratio_btw_largest_smallest_side | 0.5 | 0x1800F2070 |
| 0x18046A248 | double | g_min_elongation_ratio | 0.5 | 0x1800F2070 |
| 0x18046A250 | double | g_max_triangle_side | 1.0 (passed ×0.5) | 0x1800F2070 |
| 0x18046A258 | bool | g_unknown_bool_258 | true | none found |
| 0x18046A259 | bool | g_only_associate_a_polygon_to_a_single_plane | true | 0x1801A24D0, 0x1801A3310 |
| 0x18046A25A | bool | g_do_double_association | true | 0x18019FA90 |
| 0x18046A25B | bool | g_ground_plane_association | true | 0x1801A0AF0, 0x1801A3750 |
| 0x18046A25C | int | g_plane_min_lmk_num | 6 (used − 2) | 0x1801A0AF0 |
| 0x18046A260 | int | g_z_histogram_window_size | 2 | 0x1801A1DB0 |
| 0x18046A264 | int | g_z_histogram_max_number_of_peaks_to_select | 3 | 0x1801A1DB0 |
| 0x18046A268 | double | g_z_histogram_peak_per | 0.5 | 0x1801A1DB0 |
| 0x18046A270 | double | g_z_histogram_min_separation | 0.1 | 0x1801A1DB0 |
| 0x18046A278 | int | g_z_histogram_bins | 512 (used − 312 = 200) | 0x18019C020 |
| 0x18046A27C | int | g_hist_2d_theta_bins | 15 | 0x18019C020 (dead) |
| 0x18046A280 / 288 | double | g_z_histogram_min/max_range | −4.0 / 4.0 | 0x18019C020 |
| 0x18046A290 | int | g_hist_2d_distance_bins | 30 | 0x18019C020 (dead) |
| 0x18046A294 | int | g_plane_overlap_min_points | 3 | 0x18019E480 |
| 0x18046A298 | double | g_hist_2d_theta_range_max | 3.14 | 0x18019C020 (dead) |
| 0x18046A2A0 / 2A8 | double | g_hist_2d_distance_range_min/max | −4.0 / 4.0 | 0x18019C020 (dead) |
| 0x18046A2B0 | double | g_normal_tolerance_polygon_plane_association | 0.025 | 0x1801A0AF0 |
| 0x18046A2B8 | double | g_distance_tolerance_polygon_plane_association (c08: g_wall_distance_tolerance) | 0.15 | 0x1801A0AF0, 0x1801A1500, c08 0x1800F3430 |
| 0x18046A2C0 | double | g_normal_tolerance_horizontal_surface | 0.015 | 0x1801A0AF0, 0x18019D000 |
| 0x18046A2C8 | double | g_normal_tolerance_walls | 0.015 (+0.115 at use) | 0x1801A0AF0 |
| 0x18046A2D0 | double | g_normal_tolerance_plane_plane_association | 0.015 | 0x1801A0AF0 |
| 0x18046A2D8 | double | g_normal_tolerance_plane_plane (c08) | 0.015 | c08 0x1800EC2B0 |
| 0x18046A2E0 | double | g_distance_tolerance_plane_plane_association | 0.2 | 0x1801A0AF0 |
| 0x18046A2E8 | double | g_distance_tolerance_plane_plane (c08) | 0.1 | c08 |
| 0x18046A2F0 | double | g_plane_distance_update_tolerance (c08) | 0.15 | c08 |
| 0x18046A2F8 | int | g_max_plane_id | −1 | 0x1801A3750 (read), c08 0x1800F3430 (= max of g_new_plane_ids) |
| 0x18047EF58 | size_t | `segmentNewPlanes::plane_id` (function static) | 0 | 0x1801A24D0 |
| 0x18047EF60 | double | g_hist_2d_theta_range_min (.bss) | 0.0 | 0x18019C020 (dead) |
| 0x18047EF68 | std::vector<int> | g_new_plane_ids (atexit dtor 0x1803A6380, registered at 0x180003000 right after the mesher-object initializers 0x180002FD0..FF0) | empty | 0x1801A0AF0, 0x1801A3750, c08 0x1800F3430 |

---------------------------------------------------------------------------------------------

## 4. Constants / config defaults

Mesher ctor: z histogram `Histogram(1, {0}, Mat(), 1, {g_z_histogram_bins − 312 = 200},
{{(float)−4.0, (float)4.0}}, true, false)`; dead 2D args {15, 30}, {{0, 3.14f}, {−4, 4}},
channels {0, 1}; R_w_c_ = I, t_w_c_ = 0, lmk_px_ cleared, is_first_frame_ = true.

populate3dMesh: polygon cap 1000; INT_MIN = "no landmark"; side-ratio threshold =
min_elongation_ratio; centroid /3.0f; horizontal threshold 1 − g_2C0, relaxed to
1 − ((1 − |R22|)·0.1 + g_2C0) when |R22| < 0.7 and |d| ≥ 1.2; scale = clamp(|d|, 0.4, 1);
max side (max_side − 0.15)·s (s < 1), s·max_side, (2.0 − 1.5|R22| + max_side)·s (s ≥ 1).
isBadTriangle: ratio/side/tangential, `epsilon` (calculateNormal) 1e−3 → 0.999.
segmentPlanesInMesh: scale clamp(|d|, 0.5, 1); relax horizontal tol by +0.1 when |R22| < 0.7
and scale ≥ 1; z stored as p.z − t.z; theta += π / distance negated for theta < 0.
updatePlanesLmkIdsFromMesh: scale clamp(|d|, 0.3, 1).
updatePlanesLmkIdsFromPolygon: walls tol 0.115 / 0.3, camera-distance gate 1.5; ground gate
|distance − t.z| < 1.2 / ≥ 1.2; centroid gate 0.5·scale.
segmentHorizontalPlanes: kernel (1, 3); min_support clamp((int)(rows/10.0), 7, 20); peaks per
getLocalMaximum1D (10.0f, 1.0); distance = t.z + value.
clusterPlanesFromMesh: walls tol g_2C8 + 0.115; new-plane pruning (g_25C − 2)/scale;
associatePlanes scale clamp(|·|·0.5, 0.3, 1), wall second test (ntol + 0.1)·scale; lmk merge
margin (g_2B8 + 0.05)·scale; updatePlanesPolygon margin g_2B8 + 0.05.
ThreeDofTracker: filter gain 0.005f; gravity_ 9.80667f; init window > 0.1 s, std < 0.1;
small-angle threshold 0.008726646 (0.5°); timestamps `(uint64_t)(t·1e9)`.
ImuFilter ctor: gain 0.05f, zeta 0.001f, world frame NWU (2). GBE: 1.0 / 0.15f Hz cut-offs,
50 static frames, 0.5 / 0.03f / 0.3f thresholds, weight sum > 1000.0f.

---------------------------------------------------------------------------------------------

## 5. Quirks / bugs to preserve

1. **populate3dMesh index desync**: a landmark id missing from `points_with_id_map` breaks out
   of the triangle without advancing the running vertex index; following triangles use shifted
   landmark ids (the INT_MIN path does `index += 2 − j` correctly).
2. populate3dMesh side-ratio pre-check compares against **min_elongation_ratio** (6th arg),
   not min_ratio_largest_smallest_side.
3. populate3dMesh calls `cv::norm(normal_z)` and discards it for non-horizontal triangles
   (remnant of the wall branch). Kept as a call.
4. `printf("lmk id not equal to mesh 2d ids\n")` for every vertex whose pixel is not found
   (or maps to another id) in lmk_px_ (exact double compares of float pixels).
5. `printf("plane->lmk_ids_map size: %lu", lmk_ids_.size())` (no newline) for every new wall
   plane — note it prints lmk_ids_.size(), not the map.
6. updatePlanesLmkIdsFromPolygon **mutates its by-value tolerance parameters**: after the first
   wall plane all later planes use 0.115 / 0.3; for horizontal planes distance_tolerance is
   multiplied by distance_scale once per horizontal plane (compounding).
7. mergePolygons (0x18019CA00) uses `FromTwoVectors(+Z, normal)` (inverse of the other call
   sites) and throws away the merged hull: the backend polygon is only set when it was empty.
8. Walls: wall statistics (theta/distance Mat + polygon copies) are collected in
   segmentPlanesInMesh but never used; hist_2d_ args are built and destroyed in the ctor.
9. InitState stores the timestamp into the member `state_.t`, not into `*state` (whose t keeps
   the caller's value); Propagate then copies `*state_out` into `state_`.
10. Propagate on an empty sample vector copies `state_` into `*state_out` but keeps the
    caller's `t`.
11. PropagateGyroOnly reads `imus.back()` unconditionally (callers guarantee non-empty).
12. Mesh::operator= prints to std::cout instead of CHECK-failing on a dimension mismatch.
13. Histogram::findPeaks slope uses src(i) − src(i − size) and treats 0→1 like 2→1.
14. associatePlanes with `!g_do_double_association`: the double association is still appended
    to associated_segmented_planes / associated_plane_ids before searching further.
15. `fast_corner_detect_10_sse2` = plain detector for w ≥ 22 && h ≥ 7, plain for w < 22,
    nothing otherwise.
16. glog CHECK line numbers (FATAL messages): 796, 798 (updatePlanesLmkIdsFromMesh), 982, 983,
    999, 1010, 1011 (segmentPlanesInMesh). Kimera order of the two functions is swapped.
17. The getPolygon result is unchecked in updatePlanesLmkIdsFromMesh and
    updatePolygonMeshToTimeHorizon (stale polygon reused on failure).

---------------------------------------------------------------------------------------------

## 6. Differences vs LedObjectPoseEstimator (sensor_fusion)

* `common::Deque` / DequeHolder: identical code, path `src/sensor_fusion/deque.h`.
* GyroscopeBiasEstimator / filters / Rotation / Vector: identical except
  `kMinSumOfWeightsGyroBiasThreshold` = **1000.0f** (Led 250.0f). Namespace `pimax::ThreeDof`
  (Led `pimax::CtrlThreeDof`). `Reset()` and `GetGyroscopeBias(x,y,z)` are out of line here.
* ImuFilter: **float** (gain/zeta/q/w_b), 48 bytes (Led 88, double); keeps ROS's world-frame
  switch with an extra case 3 (gravity +X); ctor world frame NWU (Led: frame unused, gravity
  hard-coded +Y). Same 0.05 / 0.001 defaults, tracker sets gain 0.005 (same as Led).
* The tracker is a different class: float `State` (80 bytes), 32-byte `ImuSample`, no
  position/velocity integration, no acc/gyr windows, no drift check, no state history; gyro
  bias from a static initial window (or SetBias), Madgwick path when an accelerometer is
  present, otherwise gyro-only quaternion integration from the caller's state.

---------------------------------------------------------------------------------------------

## 7. Open questions / TODO(verify)

* Owning TU of the mesher globals (defined here in mesher.cpp; names partly guessed; c08 uses
  different names for some, see table).
* Histogram file path / extra Mats; `dims_` declared type.
* `LmkPixelMap` value type (Vector2d per c08; only [0]/[1] at node+32/+40 are read here).
* `LmkPositionMap` value type (Vector3f per c08; every c17 use copies 12 bytes first, also
  consistent with cv::Point3f).
* Exact source shape of the dead `cv::norm` call in populate3dMesh and of findPeaks' 0/2→1
  condition.
* ThreeDofTracker class/member names (no RTTI), +97 and +264 semantics; SetBias vs c14's
  SetImuExtrinsics (the first vector is definitely used as gyro bias).
* `#line` (or padding) to reproduce glog line numbers.
* MSVC map copy-assign codegen (clear + range insert) — check against the toolset's STL.
