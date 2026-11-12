#include "gazebo_system.hpp"

#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "gsim_core/bullet_motor.hpp"
#include "gsim_gazebo/gazebo_model.hpp"

namespace gsim {

GazeboSystem::GazeboSystem(GazeboBuffers* buffers) : buf_(buffers) {}

void GazeboSystem::Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>&,
                             gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) {
  // The depth camera sits 6 cm below the body origin, inside the body's visual mesh. OpenGL (MuJoCo,
  // Ghost's Bullet) clips what is nearer than the near plane and culls the hull's inner faces, so
  // those cameras see through it; Gazebo's depth camera reports such hits as -inf and sees the inner
  // faces once the body tilts. Seen from the camera every hull face is an inner face, so the body's
  // visuals are hidden in the server's sensor scene (the `gz sim -g` window has its own scene).
  if (buf_->hide_body_from_sensors) {
    const std::string body = buf_->model_name + "::body";   // Gazebo names scene nodes <model>::<link>
    pre_render_ = events.Connect<gz::sim::events::PreRender>([body] {   // rendering thread
      auto scene = gz::rendering::sceneFromFirstRenderEngine();
      if (auto visual = scene ? scene->VisualByName(body) : nullptr) visual->SetVisible(false);   // with its children
    });
  }
  const gz::sim::Entity model_entity = gz::sim::World(entity).ModelByName(ecm, buf_->model_name);
  if (model_entity == gz::sim::kNullEntity) return;
  gz::sim::Model model(model_entity);
  body_ = model.LinkByName(ecm, "body");
  if (body_ == gz::sim::kNullEntity) return;
  gz::sim::Link(body_).EnableVelocityChecks(ecm, true);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  joints_.assign(buf_->num_sdk_joints, gz::sim::kNullEntity);
  buf_->q = Eigen::ArrayXf::Constant(buf_->num_sdk_joints, nan);
  buf_->dq = Eigen::ArrayXf::Constant(buf_->num_sdk_joints, nan);
  for (int j = 0; j < buf_->num_sdk_joints; ++j) {
    joints_[j] = model.JointByName(ecm, std::to_string(j));   // SDK joint j <-> URDF joint "j"
    if (joints_[j] == gz::sim::kNullEntity) continue;
    gz::sim::Joint joint(joints_[j]);
    joint.EnablePositionCheck(ecm, true);
    joint.EnableVelocityCheck(ecm, true);
    buf_->q[j] = 0.f;
    buf_->dq[j] = 0.f;
  }
  buf_->configured = true;

  if (buf_->arm_joints.empty()) return;
  bool ok = true;
  for (int id : buf_->arm_joints) {
    ok = ok && id >= 0 && id < int(joints_.size()) && joints_[id] != gz::sim::kNullEntity;
    if (ok) rememberLimits(ecm, joints_[id]);
  }
  wrist_joints_.assign(5, gz::sim::kNullEntity);
  for (int i = 0; i < 5; ++i) {
    wrist_joints_[i] = model.JointByName(ecm, kWristJointNames[i]);
    if (wrist_joints_[i] == gz::sim::kNullEntity) {
      ok = false;
      continue;
    }
    gz::sim::Joint joint(wrist_joints_[i]);
    joint.EnablePositionCheck(ecm, true);
    joint.EnableVelocityCheck(ecm, true);
    if (i < 4) joint.EnableTransmittedWrenchCheck(ecm, true);
    rememberLimits(ecm, wrist_joints_[i]);
    if (const auto* axis = ecm.Component<gz::sim::components::JointAxis>(wrist_joints_[i])) {
      const auto xyz = axis->Data().Xyz();
      wrist_axis_[i] = Eigen::Vector3d(xyz.X(), xyz.Y(), xyz.Z());
      wrist_damping_[i] = axis->Data().Damping();
    }
  }
  buf_->arm_configured = ok;   // the same paused iteration's PostUpdate fills the wrist state
}

void GazeboSystem::rememberLimits(const gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint) {
  if (const auto* axis = ecm.Component<gz::sim::components::JointAxis>(joint)) {
    drives_[joint].lower = axis->Data().Lower();
    drives_[joint].upper = axis->Data().Upper();
  }
}

bool GazeboSystem::servoed(gz::sim::Entity joint) const {
  const auto d = drives_.find(joint);
  return d != drives_.end() && d->second.servo;
}

bool GazeboSystem::isArmJoint(int sdk_joint) const {
  return std::find(buf_->arm_joints.begin(), buf_->arm_joints.end(), sdk_joint) != buf_->arm_joints.end();
}

void GazeboSystem::driveForce(gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint, double tau) {
  Drive& d = drives_[joint];
  if (d.servo) {   // leaving servo: no lingering velocity command, no leftover force limit
    ecm.RemoveComponent<gz::sim::components::JointVelocityCmd>(joint);
    gz::sim::Joint(joint).SetEffortLimits(ecm, {gz::math::Vector2d(-kLiftedJointLimit, kLiftedJointLimit)});
    d.servo = false;
    d.max_force = -1.0;
  }
  gz::sim::Joint(joint).SetForce(ecm, {tau});
}

void GazeboSystem::driveServo(gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint, double q, double v_cmd, double max_force) {
  Drive& d = drives_[joint];
  if (!d.servo) {
    ecm.RemoveComponent<gz::sim::components::JointForceCmd>(joint);   // else the velocity command is ignored
    // A servo engaged on a joint that rests on its position limit never moves (DART solves the
    // limit constraint and the new servo constraint on the same row and the limit wins), even
    // when the command points back inside. Start it a hair inside the limit instead.
    constexpr double kInside = 1e-5;   // [rad]; also the tolerance for "rests on the limit"
    if (q >= d.upper - kInside && v_cmd < -0.05) gz::sim::Joint(joint).ResetPosition(ecm, {d.upper - kInside});
    if (q <= d.lower + kInside && v_cmd > 0.05) gz::sim::Joint(joint).ResetPosition(ecm, {d.lower + kInside});
  }
  if (!d.servo || d.max_force != max_force) gz::sim::Joint(joint).SetEffortLimits(ecm, {gz::math::Vector2d(-max_force, max_force)});
  d.servo = true;
  d.max_force = max_force;
  gz::sim::Joint(joint).SetVelocity(ecm, {v_cmd});
}

void GazeboSystem::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) {
  if (!buf_->configured) return;
  for (const auto& [j, q] : buf_->pending_resets) {
    if (j < 0 || j >= int(joints_.size()) || joints_[j] == gz::sim::kNullEntity) continue;
    gz::sim::Joint joint(joints_[j]);
    joint.ResetPosition(ecm, {q});
    joint.ResetVelocity(ecm, {0.0});
  }
  buf_->pending_resets.clear();
  const double dt = std::chrono::duration<double>(info.dt).count();
  if (info.paused || dt <= 0.0) return;   // a paused iteration moves nothing (and the servo divides by dt)
  const bool arm_lock = buf_->arm_configured && buf_->arm_locked;
  for (int j = 0; j < buf_->num_sdk_joints; ++j) {
    if (joints_[j] == gz::sim::kNullEntity) continue;   // SDK slots without a joint get nothing
    if (arm_lock && isArmJoint(j))   // stowed arm: Bullet's rigid motor at 0 rad, the SDK torque is not applied
      driveServo(ecm, joints_[j], buf_->q[j], bulletMotorTargetVelocity(buf_->q[j], buf_->dq[j], 0.f, 0.f, kArmLockGains, dt),
                 kArmLockMaxForce);
    else
      driveForce(ecm, joints_[j], buf_->tau[j]);
  }
  if (buf_->arm_configured) {
    const WristCommand& c = buf_->wrist_cmd;
    for (int i = 0; i < 5; ++i) {
      const int k = i < 4 ? i : 3;   // the second jaw (jwrist_g2) follows jwrist_g
      if (c.mode == WristMode::kPositionOverCurrent && c.max_current[k] > 0.f)
        driveServo(ecm, wrist_joints_[i], buf_->wrist_q5[i],
                   bulletMotorTargetVelocity(float(buf_->wrist_q5[i]), float(buf_->wrist_dq5[i]), c.pos_des[k], c.vel_des[k],
                                             kWristServoGains, dt),
                   c.max_current[k]);
      else   // mode 2; or off / a servo with no force allowed (Bullet's motor with max force 0 applies nothing)
        driveForce(ecm, wrist_joints_[i], c.mode == WristMode::kCurrent ? c.max_current[k] : 0.0);
    }
    buf_->applied_wrist = c;
  }
  if (!buf_->ext_f_B.isZero()) {
    // Body-frame force at a body-frame point -> world force and torque about the body origin,
    // where AddWorldWrench applies it (the same conversion as the MuJoCo backend).
    const Eigen::Matrix3d R = buf_->base.quat_WB.cast<double>().toRotationMatrix();
    const Eigen::Vector3d f_W = R * buf_->ext_f_B;
    const Eigen::Vector3d tau_W = (R * buf_->ext_p_B).cross(f_W);
    gz::sim::Link(body_).AddWorldWrench(ecm, gz::math::Vector3d(f_W.x(), f_W.y(), f_W.z()),
                                        gz::math::Vector3d(tau_W.x(), tau_W.y(), tau_W.z()));
  }
}

void GazeboSystem::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) {
  const double t = std::chrono::duration<double>(info.simTime).count();
  if (t < buf_->sim_time) buf_->reset_detected = true;
  buf_->sim_time = t;
  if (!buf_->configured) return;

  gz::sim::Link body(body_);
  if (auto pose = body.WorldPose(ecm)) {
    buf_->base.pos_W = Eigen::Vector3f(pose->Pos().X(), pose->Pos().Y(), pose->Pos().Z());
    buf_->base.quat_WB = Eigen::Quaternionf(pose->Rot().W(), pose->Rot().X(), pose->Rot().Y(), pose->Rot().Z());
  }
  if (auto v = body.WorldLinearVelocity(ecm)) buf_->base.vel_W = Eigen::Vector3f(v->X(), v->Y(), v->Z());
  if (auto w = body.WorldAngularVelocity(ecm)) buf_->base.angvel_W = Eigen::Vector3f(w->X(), w->Y(), w->Z());
  for (int j = 0; j < buf_->num_sdk_joints; ++j) {
    if (joints_[j] == gz::sim::kNullEntity) continue;
    gz::sim::Joint joint(joints_[j]);
    if (auto p = joint.Position(ecm); p && !p->empty()) buf_->q[j] = float((*p)[0]);
    if (auto v = joint.Velocity(ecm); v && !v->empty()) buf_->dq[j] = float((*v)[0]);
  }
  if (!buf_->arm_configured) return;
  for (int i = 0; i < 5; ++i) {
    gz::sim::Joint joint(wrist_joints_[i]);
    if (auto p = joint.Position(ecm); p && !p->empty()) buf_->wrist_q5[i] = (*p)[0];
    if (auto v = joint.Velocity(ecm); v && !v->empty()) buf_->wrist_dq5[i] = (*v)[0];
    if (i >= 4) continue;   // the SDK sees 4 wrist joints
    buf_->wrist.q[i] = float(buf_->wrist_q5[i]);
    buf_->wrist.dq[i] = float(buf_->wrist_dq5[i]);
    // The wrench the joint transmits (joint frame) includes the motor, the damping and, at a
    // position limit, the limit's reaction; its axis component minus the damping is the motor's
    // torque, what Ghost's sim reports in position mode.
    double axis = std::numeric_limits<double>::quiet_NaN();
    if (auto w = joint.TransmittedWrench(ecm); w && !w->empty()) {
      const auto& t = (*w)[0].torque();
      axis = Eigen::Vector3d(t.x(), t.y(), t.z()).dot(wrist_axis_[i]);
    }
    buf_->wrist_axis_torque[i] = axis;
    // Current as Ghost's sim reports it: the motor's torque when a servo drove the joint, the
    // commanded current in mode 2, 0 otherwise (off, or a servo allowed no force).
    if (servoed(wrist_joints_[i]))
      buf_->wrist.current[i] = float(axis + wrist_damping_[i] * buf_->wrist_dq5[i]);
    else
      buf_->wrist.current[i] = buf_->applied_wrist.mode == WristMode::kCurrent ? buf_->applied_wrist.max_current[i] : 0.f;
  }
}

}  // namespace gsim
