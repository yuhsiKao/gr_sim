// The gz::sim::System through which GazeboBackend talks to the world: commands go in at
// PreUpdate, the state comes out at PostUpdate. Everything lives in GazeboBuffers, which is only
// touched while the calling thread is inside or between Server::RunOnce calls.
//
// Each joint is driven in one of two ways per step: "force" (Joint::SetForce, the SDK's torque)
// or "servo" (Joint::SetVelocity + SetEffortLimits: DART's servo joint, the same kind of
// constraint motor as Bullet's btMultiBodyJointMotor that Ghost's sim uses for the stowed arm
// and the wrist). gz-sim ignores a velocity command while the joint
// has a force-command component, so switching drive removes the other drive's component.
#pragma once
#include <gz/common/Event.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gsim_core/sim_backend.hpp"
#include "gsim_core/toe_torsion.hpp"

namespace gsim {

struct GazeboBuffers {
  std::string model_name;
  int num_sdk_joints = 0;                               // set by loadRobot
  std::vector<double> tau;                              // size num_sdk_joints; slots without a joint ignored
  std::vector<std::pair<int, double>> pending_resets;   // (sdk joint, q), applied at the next PreUpdate
  Eigen::Vector3d ext_f_B = Eigen::Vector3d::Zero();    // external force on the body, body frame
  Eigen::Vector3d ext_p_B = Eigen::Vector3d::Zero();    // its point of application, body frame
  BaseState base;                                       // state after the last update (world frame)
  Eigen::ArrayXf q, dq;                                 // size num_sdk_joints, NaN where there is no joint
  double sim_time = 0.0;                                // [s] of the last update
  bool reset_detected = false;                          // sim time went backwards (the world was reset)
  bool configured = false;                              // Configure found the model, body and joints

  // Arm (arm_joints not empty).
  std::vector<int> arm_joints;                          // SDK ids of the arm joints, root to tip; empty = no arm
  bool arm_locked = true;                               // stowed: held at 0 rad by a servo, SDK torque not applied
  WristCommand wrist_cmd;                               // sanitized by the backend; applied at the next PreUpdate
  WristCommand applied_wrist;                           // the command the last PreUpdate applied (current rule)
  WristState wrist;                                     // the SDK's 4 wrist joints after the last update; NaN without an arm
  Eigen::Array<double, 5, 1> wrist_q5 = Eigen::Array<double, 5, 1>::Constant(std::nan(""));   // all 5 (g2 mirrors g)
  Eigen::Array<double, 5, 1> wrist_dq5 = Eigen::Array<double, 5, 1>::Constant(std::nan(""));
  Eigen::Array4d wrist_axis_torque = Eigen::Array4d::Constant(std::nan(""));   // transmitted torque about the axis (motor + damping)
  bool arm_configured = false;                          // every arm_joints id and all 5 wrist joints were found

  // Torsional friction of the toes (gsim_core/toe_torsion.hpp): DART's contacts have none.
  double toe_torsional = 0.0;                           // SimConfig::toe_friction[1]; 0 = off

  bool hide_body_from_sensors = false;                  // depth camera: the body's visuals are hidden in the server's sensor scene
};

class GazeboSystem : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate {
 public:
  explicit GazeboSystem(GazeboBuffers* buffers);
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& events) override;
  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

 private:
  struct Drive {
    bool servo = false;
    double max_force = -1.0;   // the effort limit last sent for the servo
    double lower = -std::numeric_limits<double>::infinity(), upper = std::numeric_limits<double>::infinity();   // position limits
  };
  bool isArmJoint(int sdk_joint) const;
  bool servoed(gz::sim::Entity joint) const;
  void driveForce(gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint, double tau);
  void driveServo(gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint, double q, double v_cmd, double max_force);
  void rememberLimits(const gz::sim::EntityComponentManager& ecm, gz::sim::Entity joint);
  void applyToeTorsion(gz::sim::EntityComponentManager& ecm, double dt);

  struct Toe {
    gz::sim::Entity link = gz::sim::kNullEntity, collision = gz::sim::kNullEntity;
    ToeTorsion friction;
  };

  GazeboBuffers* buf_;
  gz::sim::Entity body_ = gz::sim::kNullEntity;
  std::vector<Toe> toes_;                       // empty when toe_torsional is 0
  std::vector<gz::sim::Entity> joints_;         // size num_sdk_joints, kNullEntity for slots without a joint
  std::vector<gz::sim::Entity> wrist_joints_;   // 5 (kWristJointNames) when arm_configured
  Eigen::Vector3d wrist_axis_[5];               // joint-frame axis, for the transmitted torque's axis component
  double wrist_damping_[5] = {0, 0, 0, 0, 0};   // the URDF's joint damping
  std::unordered_map<gz::sim::Entity, Drive> drives_;   // the joints driven here: arm, wrist, legs
  gz::common::ConnectionPtr pre_render_;                 // hides the body in the sensor scene before each render
};

}  // namespace gsim
