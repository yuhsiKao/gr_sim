#include "gsim_core/depth_camera.hpp"

#include <cmath>

namespace gsim {

namespace {

// b3ComputeProjectionMatrixFOV (Bullet): the standard OpenGL perspective matrix.
Eigen::Matrix4f projection() {
  using C = FactoryDepthCamera;
  const float y_scale = 1.f / C::tanHalfFovy();
  const float x_scale = y_scale / C::kAspect;
  Eigen::Matrix4f p = Eigen::Matrix4f::Zero();
  p(0, 0) = x_scale;
  p(1, 1) = y_scale;
  p(2, 2) = (C::kFar + C::kNear) / (C::kNear - C::kFar);
  p(2, 3) = 2.f * C::kFar * C::kNear / (C::kNear - C::kFar);
  p(3, 2) = -1.f;
  return p;
}

}  // namespace

Eigen::Affine3f FactoryDepthCamera::bodyFromCamera() {
  return Eigen::Translation3f(0.f, 0.f, -0.06f) * Eigen::AngleAxisf(0.5f * float(M_PI), Eigen::Vector3f::UnitZ());
}

float FactoryDepthCamera::tanHalfFovy() { return std::tan(0.5f * kFovyDeg * float(M_PI) / 180.f); }

float glDepthFromLinear(float z_m) {
  using C = FactoryDepthCamera;
  if (std::isnan(z_m) || z_m >= C::kFar) return 1.f;
  if (z_m <= C::kNear) return 0.f;
  // The projection() matrix applied to the eye-space point (0, 0, -z_m): ndc z = (-p22 z + p23) / z.
  static const Eigen::Matrix4f p = projection();
  return 0.5f + 0.5f * (-p(2, 2) + p(2, 3) / z_m);
}

void depthToBodyCloud(const float* depth, PointCloud& cloud_B) {
  using C = FactoryDepthCamera;
  static const Eigen::Matrix4f p_inv = projection().inverse();
  static const Eigen::Affine3f b_T_c = C::bodyFromCamera();
  cloud_B.resize(C::kWidth * C::kHeight, 3);
  int k = 0;
  for (int i = 0; i < C::kHeight; ++i) {
    for (int j = 0; j < C::kWidth; ++j, ++k) {
      // NDC as SimWrapper::initFirstTime; "depth" = z/w with w = 1 (SimWrapper::depthToCloud)
      const Eigen::Vector4f ndc(-2.f * j / C::kWidth + 1.f, 2.f * i / C::kHeight - 1.f, 2.f * depth[k] - 1.f, 1.f);
      const Eigen::Vector4f h = p_inv * ndc;
      cloud_B.row(k) = (b_T_c * (h.head<3>() / h.w())).transpose();
    }
  }
}

}  // namespace gsim
