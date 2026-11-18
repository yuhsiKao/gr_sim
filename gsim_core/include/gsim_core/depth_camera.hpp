// Ghost's synthetic depth camera, as its Bullet simulator renders it for the SDK's terrain
// perception (vision_mode with cameras on): one downward-looking camera under the body whose
// point cloud, in the body frame, feeds the SDK's heightmaps.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gsim {

// Same type as the SDK's PointCloud_t (SMath.h): one row per point, body frame.
using PointCloud = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

struct FactoryDepthCamera {
  static constexpr int kWidth = 75;            // depth_aspect_ * height (SimWrapper.cpp getCameraImage2)
  static constexpr int kHeight = 1500;         // SimWrapper.cpp getCameraImage2
  static constexpr int kStepsPerFrame = 66;    // PERC_DEC_ = (uint32_t)(2000 / CAM_RATE_ = 30) (SimWrapper.h)
  static constexpr float kFovyDeg = 170.f;     // depth_fov_ (SimWrapper.cpp constructor)
  static constexpr float kAspect = float(kWidth) / kHeight;   // depth_aspect_ = 0.05
  static constexpr float kNear = 0.05f;        // cam_near_
  static constexpr float kFar = 10.f;          // cam_far_
  // bTc_depth_ = Translation(0, 0, -0.06) * RotZ(pi/2) (SimWrapper.h depth_trans_, SimWrapper.cpp init):
  // the camera sits 6 cm below the body origin and looks along the body's -z, image up = body +x.
  static Eigen::Affine3f bodyFromCamera();
  static float tanHalfFovy();   // tan(kFovyDeg / 2)
};

// Ghost's SimWrapper::perceptionUpdate renders a new frame every kStepsPerFrame calls, the first
// on the kStepsPerFrame-th. Counts one call in `calls` and says whether this one is a frame.
inline bool depthFrameDue(long& calls) {
  calls = (calls + 1) % FactoryDepthCamera::kStepsPerFrame;
  return calls == 0;
}

// depth: kWidth * kHeight standard OpenGL depth values (0 = near plane, 1 = far plane) in Ghost's
// pixel order (k = i * kWidth + j, row i = 0 at the top of the image). Writes one body-frame point per
// pixel into cloud_B, exactly as SimWrapper::initFirstTime + depthToCloud do.
void depthToBodyCloud(const float* depth, PointCloud& cloud_B);

// A linear depth along the optical axis [m] (what Gazebo's depth camera gives) as the OpenGL depth
// value above: 0 at the near plane, 1 at the far plane; beyond far, +inf and NaN -> 1, inside near and -inf -> 0.
float glDepthFromLinear(float z_m);

}  // namespace gsim
