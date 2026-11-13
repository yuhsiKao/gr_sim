#include "gsim_mujoco/model_builder.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>

#include "gsim_core/bullet_motor.hpp"
#include "gsim_core/depth_camera.hpp"

namespace gsim {

namespace {

namespace fs = std::filesystem;
using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;

std::string readFile(const std::string& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

// MuJoCo's default log handler appends to MUJOCO_LOG.TXT in the working directory;
// collect warnings instead while the model is being built.
std::vector<std::string>* g_warning_sink = nullptr;
mjfLogHandler g_previous_handler = nullptr;

void captureLog(const mjLogMessage* msg) {
  if (msg->level == mjLOG_WARNING && g_warning_sink)
    g_warning_sink->push_back(msg->subject);
  else if (msg->level >= mjLOG_WARNING && g_previous_handler)
    g_previous_handler(msg);
}

struct ScopedWarningCapture {
  explicit ScopedWarningCapture(std::vector<std::string>* sink) {
    g_warning_sink = sink;
    g_previous_handler = mju_setLogHandler(captureLog);
  }
  ~ScopedWarningCapture() {
    mju_setLogHandler(g_previous_handler);
    g_warning_sink = nullptr;
    g_previous_handler = nullptr;
  }
};

std::string elementName(mjsElement* e) {
  const char* s = mjs_getString(mjs_getName(e));
  return s ? s : "";
}

const char* nameOf(const mjModel* m, mjtObj type, int id) {
  const char* s = mj_id2name(m, type, id);
  return s ? s : "";
}

// Robot geoms collide with the world only (never with each other), matching
// Bullet's URDF loading without URDF_USE_SELF_COLLISION. Visual geoms arrive from
// the URDF importer as group 1 with contype = conaffinity = 0 and are left alone.
void configureGeoms(mjsBody* base, const ModelOptions& opt) {
  for (mjsElement* e = mjs_firstChild(base, mjOBJ_GEOM, 1); e; e = mjs_nextChild(base, e, 1)) {
    mjsGeom* g = mjs_asGeom(e);
    if (g->contype == 0 && g->conaffinity == 0) continue;
    g->contype = 1;
    g->conaffinity = 0;
    if (startsWith(elementName(mjs_getParent(e)->element), "toe")) {
      // condim 4 turns the torsional coefficient on: Ghost's Bullet toes have spinning_friction,
      // and the legs have no vertical-axis joint, so a toe that cannot spin holds the body's yaw.
      g->condim = kToeCondim;
      for (int i = 0; i < 3; ++i) g->friction[i] = opt.toe_friction[i];
      for (int i = 0; i < 2; ++i) g->solref[i] = opt.toe_solref[i];
    }
  }
}

// URDF effort limits become joint actuatorfrcrange; SimLLC already limits torques.
void removeJointForceLimits(mjsBody* base) {
  for (mjsElement* e = mjs_firstChild(base, mjOBJ_JOINT, 1); e; e = mjs_nextChild(base, e, 1))
    mjs_asJoint(e)->actfrclimited = mjLIMITED_FALSE;
}

mjsActuator* addTorqueActuator(mjSpec* spec, const std::string& name, const std::string& joint) {
  mjsActuator* a = mjs_addActuator(spec, nullptr);
  mjs_setName(a->element, name.c_str());
  a->trntype = mjTRN_JOINT;
  mjs_setString(a->target, joint.c_str());
  a->gaintype = mjGAIN_FIXED;
  a->gainprm[0] = 1.0;
  a->biastype = mjBIAS_NONE;
  a->dyntype = mjDYN_NONE;
  a->gear[0] = 1.0;
  a->ctrllimited = mjLIMITED_FALSE;
  return a;
}

// Velocity servo: force = kv * (ctrl - qvel), ctrl = target velocity, force limited. Built switched
// off (kv = 0); the backend sets kv, the target and the force limit before every step.
void addTrackerActuator(mjSpec* spec, const std::string& name, const std::string& joint) {
  mjsActuator* a = addTorqueActuator(spec, name, joint);
  a->gainprm[0] = 0.0;
  a->biastype = mjBIAS_AFFINE;   // bias = biasprm[2] * qvel, with biasprm[2] = -kv
  a->biasprm[0] = a->biasprm[1] = a->biasprm[2] = 0.0;
  a->forcelimited = mjLIMITED_TRUE;
  a->forcerange[0] = -1.0;       // placeholder, set every step
  a->forcerange[1] = 1.0;
}

void addMotors(mjSpec* spec, int num_sdk_joints) {
  for (int j = 0; j < num_sdk_joints; ++j) {
    const std::string joint = std::to_string(j);
    if (!mjs_findElement(spec, mjOBJ_JOINT, joint.c_str())) continue;
    addTorqueActuator(spec, "m" + joint, joint);
  }
}

void addArmActuators(mjSpec* spec, const std::vector<int>& arm_joints) {
  for (int id : arm_joints) {
    const std::string joint = std::to_string(id);
    if (mjs_findElement(spec, mjOBJ_JOINT, joint.c_str()))
      addTrackerActuator(spec, "lock" + joint, joint);
  }
  for (const char* w : kWristJointNames) {
    if (!mjs_findElement(spec, mjOBJ_JOINT, w)) continue;
    addTrackerActuator(spec, std::string("ws_") + w, w);
    addTorqueActuator(spec, std::string("wt_") + w, w);
  }
}

// Ghost's synthetic depth camera (FactoryDepthCamera): 6 cm below the body origin, looking along the
// body's -z with the image top towards the body's +x. A MuJoCo camera looks along its own -z with its
// +y up, so its frame is the body frame turned by -90 deg about z.
void addDepthCamera(mjsBody* base) {
  mjsCamera* cam = mjs_addCamera(base, nullptr);
  mjs_setName(cam->element, kDepthCameraName);
  const Eigen::Vector3f pos = FactoryDepthCamera::bodyFromCamera().translation();
  for (int i = 0; i < 3; ++i) cam->pos[i] = pos[i];
  const double h = std::sqrt(0.5);
  cam->quat[0] = h;
  cam->quat[1] = 0.0;
  cam->quat[2] = 0.0;
  cam->quat[3] = -h;
  cam->fovy = FactoryDepthCamera::kFovyDeg;
}

// Facts read from the URDF text, for the arm checks.
double urdfTotalMass(const std::string& text) {
  double total = 0.0;
  for (size_t pos = text.find("<mass"); pos != std::string::npos; pos = text.find("<mass", pos + 1)) {
    const size_t v = text.find("value=\"", pos);
    if (v != std::string::npos) total += std::atof(text.c_str() + v + 7);
  }
  return total;
}

// The meshes MuJoCo's URDF importer makes: one per distinct file. vision60_arm.urdf uses
// cp20_finger_nofoam.stl for the visual and .obj for the collision; MuJoCo 4 keeps both, while
// MuJoCo 3 named meshes after the file stem and merged them (the .stl for both).
int urdfDistinctMeshes(const std::string& text) {
  std::set<std::string> files;
  for (size_t pos = text.find("<mesh"); pos != std::string::npos; pos = text.find("<mesh", pos + 1)) {
    const size_t f = text.find("filename=\"", pos);
    if (f == std::string::npos) continue;
    const size_t end = text.find('"', f + 10);
    const fs::path file(text.substr(f + 10, end - f - 10));
#if mjVERSION_HEADER >= 4000000
    files.insert(file.filename().string());
#else
    files.insert(file.stem().string());
#endif
  }
  return static_cast<int>(files.size());
}

}  // namespace

BuiltModel buildMujocoModel(const ModelOptions& opt) {
  BuiltModel out;
  ScopedWarningCapture capture(&out.warnings);
  if (!fs::exists(opt.urdf_path)) {
    out.error = "URDF not found: " + opt.urdf_path;
    return out;
  }
  const std::string urdf_dir = fs::path(opt.urdf_path).parent_path().string();

  // Keep toe bodies (fusestatic=false) and visual meshes; never written back to disk.
  std::string urdf = readFile(opt.urdf_path);
  const size_t tag = urdf.find("<robot");
  const size_t close = tag == std::string::npos ? std::string::npos : urdf.find('>', tag);
  if (close == std::string::npos) {
    out.error = "no <robot> element in " + opt.urdf_path;
    return out;
  }
  urdf.insert(close + 1, "<mujoco><compiler fusestatic=\"false\" discardvisual=\"false\" meshdir=\"" + urdf_dir +
                             "/\"/></mujoco>");

  char err[1024] = "";
  SpecPtr robot(mj_parseXMLString(urdf.c_str(), nullptr, err, sizeof(err)), &mj_deleteSpec);
  if (!robot) {
    out.error = "failed to parse " + opt.urdf_path + ": " + err;
    return out;
  }
  for (mjsElement* e = mjs_firstElement(robot.get(), mjOBJ_MESH); e; e = mjs_nextElement(robot.get(), e)) {
    mjsMesh* mesh = mjs_asMesh(e);
    const std::string file = mjs_getString(mesh->file);
    if (!file.empty() && fs::path(file).is_relative())
      mjs_setString(mesh->file, (fs::path(urdf_dir) / file).string().c_str());
  }

  SpecPtr scene(mj_parseXML(opt.scene_path.c_str(), nullptr, err, sizeof(err)), &mj_deleteSpec);
  if (!scene) {
    out.error = "failed to parse scene " + opt.scene_path + ": " + err;
    return out;
  }

  mjsBody* robot_root = mjs_findBody(robot.get(), "body");
  if (!robot_root) {
    out.error = "URDF " + opt.urdf_path + " has no link named 'body'";
    return out;
  }
  // MuJoCo: a body can only be attached to a frame (or site), not directly to a body.
  mjsFrame* mount = mjs_addFrame(mjs_findBody(scene.get(), "world"), nullptr);
  if (!mjs_attach(mount->element, robot_root->element, "", "")) {
    out.error = std::string("failed to attach robot to scene: ") + mjs_getError(scene.get());
    return out;
  }

  mjsBody* base = mjs_findBody(scene.get(), "body");
  for (int i = 0; i < 3; ++i) base->pos[i] = opt.base_pos[i];
  base->quat[0] = opt.base_quat.w();
  base->quat[1] = opt.base_quat.x();
  base->quat[2] = opt.base_quat.y();
  base->quat[3] = opt.base_quat.z();
  if (!opt.fixed_base) mjs_addFreeJoint(base);
  configureGeoms(base, opt);
  removeJointForceLimits(base);
  addMotors(scene.get(), opt.num_sdk_joints);
  addDepthCamera(base);
  if (!opt.arm_joints.empty()) addArmActuators(scene.get(), opt.arm_joints);

  out.model = mj_compile(scene.get(), nullptr);
  if (!out.model) {
    out.error = std::string("MuJoCo compile failed: ") + mjs_getError(scene.get());
    return out;
  }

  const mjModel* m = out.model;
  out.base_body = mj_name2id(m, mjOBJ_BODY, "body");
  out.depth_camera = mj_name2id(m, mjOBJ_CAMERA, kDepthCameraName);
  out.joints.qposadr.assign(opt.num_sdk_joints, -1);
  out.joints.dofadr.assign(opt.num_sdk_joints, -1);
  out.joints.actuator.assign(opt.num_sdk_joints, -1);
  for (int j = 0; j < opt.num_sdk_joints; ++j) {
    const std::string joint = std::to_string(j);
    const int id = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
    if (id < 0) continue;
    out.joints.qposadr[j] = m->jnt_qposadr[id];
    out.joints.dofadr[j] = m->jnt_dofadr[id];
    out.joints.actuator[j] = mj_name2id(m, mjOBJ_ACTUATOR, ("m" + joint).c_str());
  }
  if (!opt.arm_joints.empty()) {
    out.arm.joints = opt.arm_joints;
    for (int id : opt.arm_joints)
      out.arm.lock_actuator.push_back(mj_name2id(m, mjOBJ_ACTUATOR, ("lock" + std::to_string(id)).c_str()));
    out.arm.has_wrist = true;
    for (int i = 0; i < 5; ++i) {
      const int jnt = mj_name2id(m, mjOBJ_JOINT, kWristJointNames[i]);
      if (jnt < 0) {
        out.arm.has_wrist = false;
        continue;
      }
      out.arm.wrist_qposadr[i] = m->jnt_qposadr[jnt];
      out.arm.wrist_dofadr[i] = m->jnt_dofadr[jnt];
      out.arm.wrist_servo[i] = mj_name2id(m, mjOBJ_ACTUATOR, (std::string("ws_") + kWristJointNames[i]).c_str());
      out.arm.wrist_torque[i] = mj_name2id(m, mjOBJ_ACTUATOR, (std::string("wt_") + kWristJointNames[i]).c_str());
    }
  }
  return out;
}

std::vector<std::string> checkMujocoModel(const mjModel* m, const JointMap& j, const ModelOptions& opt) {
  std::vector<std::string> fail;
  auto add = [&fail](const std::string& msg) { fail.push_back(msg); };
  // vision60.urdf has the 12 leg joints (SDK ids 0..11). The SDK may report more
  // joints (15 on v5: arm slots 12..14) which are simply absent from the model.
  constexpr int kLegJoints = 12;
  const bool arm = !opt.arm_joints.empty();
  // With an arm (vision60_arm.urdf) every SDK joint and the 5 wrist joints are in the model.
  const int n = arm ? opt.num_sdk_joints : kLegJoints;
  const int n_wrist = arm ? 5 : 0;
  if (opt.num_sdk_joints < kLegJoints)
    add("joint: num_sdk_joints=" + std::to_string(opt.num_sdk_joints) + " is smaller than the 12 leg joints");
  const int base = mj_name2id(m, mjOBJ_BODY, "body");
  if (base < 0) {
    add("joint: no body named 'body'");
    return fail;
  }

  const long long n_jnt = n + n_wrist;
  const long long nq_expect = opt.fixed_base ? n_jnt : n_jnt + 7, nv_expect = opt.fixed_base ? n_jnt : n_jnt + 6;
  if (m->nq != nq_expect || m->nv != nv_expect)
    add("nq/nv: got " + std::to_string(m->nq) + "/" + std::to_string(m->nv) + ", expected " +
        std::to_string(nq_expect) + "/" + std::to_string(nv_expect));
  const int nu_expect = arm ? opt.num_sdk_joints + static_cast<int>(opt.arm_joints.size()) + 2 * n_wrist : n;
  if (m->nu != nu_expect)
    add("nu: got " + std::to_string(m->nu) + " actuators, expected " + std::to_string(nu_expect));
  for (int k = 0; k < n; ++k)
    if (k >= static_cast<int>(j.qposadr.size()) || j.qposadr[k] < 0 || j.actuator[k] < 0)
      add("joint: SDK joint " + std::to_string(k) + " missing from model or without actuator");
  if (arm) {
    auto onJoint = [m](const std::string& actuator, const std::string& joint) {
      const int a = mj_name2id(m, mjOBJ_ACTUATOR, actuator.c_str());
      const int jt = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
      return a >= 0 && jt >= 0 && m->actuator_trntype[a] == mjTRN_JOINT && m->actuator_trnid[2 * a] == jt;
    };
    for (int id : opt.arm_joints) {
      const std::string joint = std::to_string(id);
      if (!onJoint("lock" + joint, joint)) add("arm: actuator lock" + joint + " missing or not on joint " + joint);
    }
    for (const char* w : kWristJointNames) {
      if (mj_name2id(m, mjOBJ_JOINT, w) < 0) {
        add(std::string("arm: wrist joint ") + w + " missing from model");
        continue;
      }
      for (const char* prefix : {"ws_", "wt_"})
        if (!onJoint(prefix + std::string(w), w))
          add(std::string("arm: actuator ") + prefix + w + " missing or not on joint " + w);
    }
  }

  const std::string urdf = arm ? readFile(opt.urdf_path) : std::string();
  const double mass_expect = arm ? urdfTotalMass(urdf) : 45.1;
  const int meshes_expect = arm ? urdfDistinctMeshes(urdf) : 5;
  double mass = 0.0;
  for (int b = 0; b < m->nbody; ++b)
    if (m->body_rootid[b] == base) mass += m->body_mass[b];
  if (std::fabs(mass - mass_expect) > 0.01)
    add("mass: robot mass " + std::to_string(mass) + " kg, expected " + std::to_string(mass_expect));
  if (m->nmesh != meshes_expect)
    add("mesh: got " + std::to_string(m->nmesh) + " meshes, expected " + std::to_string(meshes_expect));

  const struct { const char* name; double lo, hi; } ranges[] = {{"1", 0.3, 2.6}, {"0", -0.43, 5.4}, {"8", -0.56, 0.56}};
  for (const auto& r : ranges) {
    const int id = mj_name2id(m, mjOBJ_JOINT, r.name);
    if (id < 0 || std::fabs(m->jnt_range[2 * id] - r.lo) > 1e-6 || std::fabs(m->jnt_range[2 * id + 1] - r.hi) > 1e-6)
      add(std::string("range: joint ") + r.name + " range differs from URDF (radians expected)");
  }
  for (int k = 0; k < m->njnt; ++k)
    if (m->body_rootid[m->jnt_bodyid[k]] == base && m->jnt_actfrclimited[k])
      add(std::string("actfrclimited: joint ") + nameOf(m, mjOBJ_JOINT, k) + " has an actuator force limit");

  const int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
  if (floor < 0) add("floor: scene has no geom named 'floor'");
  auto collides = [m](int a, int b) {
    return (m->geom_contype[a] & m->geom_conaffinity[b]) || (m->geom_contype[b] & m->geom_conaffinity[a]);
  };
  std::vector<int> robot_geoms;
  int toes = 0;
  for (int g = 0; g < m->ngeom; ++g) {
    // Without an arm every mesh is a visual. vision60_arm.urdf also has collision meshes (the
    // gripper jaws), so with an arm only the importer's visual group (1) must not collide.
    const bool visual = !arm || m->geom_group[g] == 1;
    if (m->geom_type[g] == mjGEOM_MESH && visual && (m->geom_contype[g] || m->geom_conaffinity[g]))
      add(std::string("visual: mesh geom on body ") + nameOf(m, mjOBJ_BODY, m->geom_bodyid[g]) + " can collide");
    if (m->body_rootid[m->geom_bodyid[g]] != base) continue;
    robot_geoms.push_back(g);
    const std::string body = nameOf(m, mjOBJ_BODY, m->geom_bodyid[g]);
    if (!startsWith(body, "toe") || !(m->geom_contype[g] || m->geom_conaffinity[g])) continue;
    ++toes;
    if (floor >= 0 && !collides(g, floor)) add("floor: toe geom on " + body + " does not collide with the floor");
    for (int i = 0; i < 3; ++i)
      if (std::fabs(m->geom_friction[3 * g + i] - opt.toe_friction[i]) > 1e-9)
        add("friction: toe geom on " + body + " friction differs from the requested value");
    if (m->geom_condim[g] != kToeCondim)
      add("condim: toe geom on " + body + " has no torsional friction (condim " + std::to_string(m->geom_condim[g]) + ")");
  }
  if (toes != 4) add("toe: found " + std::to_string(toes) + " toe collision geoms, expected 4");
  for (size_t a = 0; a < robot_geoms.size(); ++a)
    for (size_t b = a + 1; b < robot_geoms.size(); ++b)
      if (collides(robot_geoms[a], robot_geoms[b])) {
        add(std::string("self-collision: robot geoms on ") + nameOf(m, mjOBJ_BODY, m->geom_bodyid[robot_geoms[a]]) +
            " and " + nameOf(m, mjOBJ_BODY, m->geom_bodyid[robot_geoms[b]]) + " can collide");
        a = robot_geoms.size();  // one message is enough
        break;
      }

  if (std::fabs(m->opt.timestep - 0.0005) > 1e-12)
    add("timestep: " + std::to_string(m->opt.timestep) + " s, expected 0.0005");
  if (m->opt.integrator != mjINT_IMPLICITFAST) add("integrator: expected implicitfast");
  return fail;
}

}  // namespace gsim
