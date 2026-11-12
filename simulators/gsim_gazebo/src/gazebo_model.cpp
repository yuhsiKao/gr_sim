#include "gsim_gazebo/gazebo_model.hpp"

#include <sdf/sdf.hh>

#include "gsim_core/depth_camera.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <fstream>
#include <regex>
#include <sstream>

namespace gsim {

namespace {

std::string num(double v) {
  std::ostringstream s;
  s << v;
  return s.str();
}

void setError(std::string* error, const std::string& msg) {
  if (error) *error = msg;
}

// Collision meshes given as .obj -> the .stl next to it when there is one (mesh paths are
// already absolute file:// URIs here).
void preferStlCollisionMeshes(std::string& urdf) {
  const std::regex obj("filename=\"file://([^\"]*)\\.obj\"");
  for (size_t c = urdf.find("<collision"); c != std::string::npos; c = urdf.find("<collision", c + 1)) {
    const size_t end = urdf.find("</collision>", c);
    if (end == std::string::npos) break;
    const std::string block = urdf.substr(c, end - c);
    std::string out;
    auto begin = block.cbegin();
    for (std::smatch m; std::regex_search(begin, block.cend(), m, obj); begin = m[0].second) {
      out.append(begin, m[0].first);
      const std::string stem = m[1];
      out += std::filesystem::exists(stem + ".stl") ? "filename=\"file://" + stem + ".stl\"" : m[0].str();
    }
    out.append(begin, block.cend());
    urdf.replace(c, end - c, out);
  }
}

// Gazebo clips by straight-line distance, OpenGL by depth along the optical axis: reach an axial
// kFar in every pixel (the image corner is the farthest, about 114.9 m); glDepthFromLinear then
// cuts at an axial kFar.
double depthCameraFarClip() {
  using Cam = FactoryDepthCamera;
  const double tv = Cam::tanHalfFovy(), th = tv * Cam::kAspect;
  return Cam::kFar * std::sqrt(1.0 + tv * tv + th * th);
}

}  // namespace

std::string prepareUrdf(const GazeboModelOptions& opt, std::string* error) {
  std::ifstream f(opt.urdf_path);
  if (!f) {
    setError(error, "cannot read the URDF " + opt.urdf_path);
    return "";
  }
  std::stringstream ss;
  ss << f.rdbuf();
  std::string urdf = ss.str();

  // Mesh files are named relative to the URDF; sdformat resolves file:// URIs.
  const std::string dir = opt.urdf_path.substr(0, opt.urdf_path.find_last_of('/') + 1);
  urdf = std::regex_replace(urdf, std::regex("filename=\"([^\"/][^\"]*)\""), "filename=\"file://" + dir + "$1\"");

  preferStlCollisionMeshes(urdf);

  // The SDK limits its torques itself and the other backends enforce neither limit: lift them
  // on every revolute joint (legs, arm, wrist) so that DART does not clamp the torque or the
  // speed; the arm lock and the wrist servo set their own force limits per step.
  const std::regex limit("(<joint name=\"[^\"]*\" type=\"revolute\">[\\s\\S]*?<limit[^>]*?)effort=\"[^\"]*\"([^>]*?)velocity=\"[^\"]*\"");
  const std::string lifted = std::to_string(static_cast<long>(kLiftedJointLimit));
  urdf = std::regex_replace(urdf, limit, "$1effort=\"" + lifted + "\"$2velocity=\"" + lifted + "\"");

  std::string tags;
  for (int i = 0; i < 4; ++i)
    tags += "  <gazebo reference=\"toe" + std::to_string(i) + "\"><mu1>" + num(opt.toe_mu) + "</mu1><mu2>" + num(opt.toe_mu) +
            "</mu2></gazebo>\n";
  tags += "  <gazebo><self_collide>false</self_collide></gazebo>\n</robot>";
  const size_t end = urdf.rfind("</robot>");
  if (end == std::string::npos) {
    setError(error, opt.urdf_path + " has no </robot>");
    return "";
  }
  urdf.replace(end, 8, tags);
  return urdf;
}

std::string urdfToSdfModel(const std::string& urdf, const GazeboModelOptions& opt, std::string* error) {
  sdf::SDFPtr sdfp(new sdf::SDF());
  sdf::init(sdfp);
  sdf::Errors errs;
  if (!sdf::readString(urdf, sdf::ParserConfig::GlobalConfig(), sdfp, errs) || !sdfp->Root()->HasElement("model")) {
    std::string msg = "URDF to SDF conversion failed";
    for (const auto& e : errs) msg += "; " + e.Message();
    setError(error, msg);
    return "";
  }
  sdf::ElementPtr model = sdfp->Root()->GetElement("model");
  model->GetAttribute("name")->Set<std::string>(opt.model_name);
  const std::string pose = num(opt.pos0_W.x()) + " " + num(opt.pos0_W.y()) + " " + num(opt.pos0_W.z()) + " " +
                           num(opt.rpy0.x()) + " " + num(opt.rpy0.y()) + " " + num(opt.rpy0.z());
  const bool had_pose = model->HasElement("pose");   // a URDF never has one; sdformat would append it last
  if (had_pose) model->GetElement("pose")->Set<std::string>(pose);
  if (opt.fixed_base) {
    sdf::ElementPtr joint = model->AddElement("joint");
    joint->GetAttribute("name")->Set<std::string>("world_fixed");
    joint->GetAttribute("type")->Set<std::string>("fixed");
    joint->GetElement("parent")->Set<std::string>("world");
    joint->GetElement("child")->Set<std::string>("body");
  }
  std::string out = model->ToString("");
  if (!had_pose) out.insert(out.find('>') + 1, "\n  <pose>" + pose + "</pose>");   // first child, as SDF is usually written
  if (opt.depth_camera) {
    const std::string body = "<link name='body'>";
    const size_t at = out.find(body);
    if (at == std::string::npos) {
      setError(error, "the converted model has no link 'body' for the depth camera");
      return "";
    }
    out.insert(at + body.size(), "\n" + depthCameraSensorSdf(opt.depth_topic, opt.timestep));
  }
  return out;
}


std::string depthCameraSensorSdf(const std::string& topic, double timestep_s) {
  using Cam = FactoryDepthCamera;
  // Gazebo's camera looks along its +x with image up = +z; the OpenGL camera of bodyFromCamera()
  // looks along -z with up = +y. Rotating the sensor by +90 deg about y makes it look along the
  // body's -z with image up = body +x, which with depthToBodyCloud's pixel order (row 0 at the top,
  // no flipping) matches Ghost's camera (checked against a box scene, all four flips compared).
  const Eigen::Vector3f t = Cam::bodyFromCamera().translation();
  std::ostringstream s;
  s << std::setprecision(9) << "<sensor name='depth_underside' type='depth_camera'>"
    << "<pose>" << t.x() << " " << t.y() << " " << t.z() << " 0 " << 0.5 * M_PI << " 0</pose>"
    << "<always_on>1</always_on><visualize>0</visualize>"
    << "<update_rate>" << 1.0 / (Cam::kStepsPerFrame * timestep_s) << "</update_rate><topic>" << topic << "</topic>"
    << "<camera><horizontal_fov>" << 2.0 * std::atan(double(Cam::tanHalfFovy()) * Cam::kAspect) << "</horizontal_fov>"
    << "<image><width>" << Cam::kWidth << "</width><height>" << Cam::kHeight << "</height><format>R_FLOAT32</format></image>"
    << "<clip><near>" << Cam::kNear << "</near><far>" << depthCameraFarClip() << "</far></clip></camera></sensor>";
  return s.str();
}

std::string worldWithSensorsSystem(const std::string& world_sdf) {
  if (world_sdf.find("gz::sim::systems::Sensors") != std::string::npos) return world_sdf;
  const std::string plugin =
      "<plugin filename=\"gz-sim-sensors-system\" name=\"gz::sim::systems::Sensors\"><render_engine>ogre2</render_engine></plugin>\n";
  std::string out = world_sdf;
  size_t at = out.find("<plugin");
  if (at != std::string::npos) {
    out.insert(at, plugin);
    return out;
  }
  // A world without plugins gets Gazebo's defaults (server.config); one plugin would stop that,
  // so they are listed too.
  at = out.rfind("</world>");
  if (at == std::string::npos) return world_sdf;
  out.insert(at, "<plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\"/>\n"
                 "<plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>\n"
                 "<plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n" + plugin);
  return out;
}

std::string worldWithModel(const std::string& world_sdf, const std::string& model_sdf) {
  const size_t end = world_sdf.rfind("</world>");
  if (end == std::string::npos) return world_sdf;
  return world_sdf.substr(0, end) + model_sdf + world_sdf.substr(end);
}

std::vector<std::string> checkGazeboModel(const std::string& model_sdf, double toe_mu, const std::vector<int>& arm_joints,
                                          bool depth_camera) {
  std::vector<std::string> problems;
  auto missing = [&](const std::string& joint) {
    return model_sdf.find("<joint name='" + joint + "' type='revolute'>") == std::string::npos;
  };
  for (int j = 0; j < 12; ++j)
    if (missing(std::to_string(j))) problems.push_back("joint \"" + std::to_string(j) + "\" is missing from the converted model");
  int toes = 0, friction = 0;
  for (int i = 0; i < 4; ++i)
    toes += model_sdf.find("fixed_joint_lump__toe" + std::to_string(i) + "_collision_1") != std::string::npos;
  if (toes != 4) problems.push_back("expected 4 toe collisions, found " + std::to_string(toes));
  const std::regex mu("<mu>([0-9.eE+-]+)</mu>");
  for (auto it = std::sregex_iterator(model_sdf.begin(), model_sdf.end(), mu); it != std::sregex_iterator(); ++it)
    friction += std::fabs(std::stod((*it)[1]) - toe_mu) < 1e-6;
  if (friction < 4) problems.push_back("toe friction " + num(toe_mu) + " found on " + std::to_string(friction) + " of 4 toes");
  for (int id : arm_joints)
    if (missing(std::to_string(id))) problems.push_back("arm joint \"" + std::to_string(id) + "\" is missing from the converted model");
  if (!arm_joints.empty())
    for (const char* w : kWristJointNames)
      if (missing(w)) problems.push_back(std::string("wrist joint ") + w + " is missing from the converted model");
  if (depth_camera) {
    const size_t body = model_sdf.find("<link name='body'>");
    const size_t end = body == std::string::npos ? std::string::npos : model_sdf.find("</link>", body);
    const size_t cam = body == std::string::npos ? std::string::npos : model_sdf.find("type='depth_camera'", body);
    const std::string size = "<width>" + std::to_string(FactoryDepthCamera::kWidth) + "</width><height>" +
                             std::to_string(FactoryDepthCamera::kHeight) + "</height>";
    if (cam == std::string::npos || cam > end || model_sdf.find(size, cam) > end)
      problems.push_back("the depth camera (75 x 1500 depth_camera sensor) is missing from the body link");
  }
  // Collision meshes must not be .obj files (prepareUrdf swaps them for the .stl): the gripper's
  // has no normals, DART drops it and ODE aborts.
  for (size_t c = model_sdf.find("<collision"); c != std::string::npos; c = model_sdf.find("<collision", c + 1)) {
    const size_t obj = model_sdf.find(".obj</uri>", c);
    if (obj != std::string::npos && obj < model_sdf.find("</collision>", c)) {
      problems.push_back("a collision mesh is an .obj file (DART drops a mesh without normals and ODE aborts)");
      break;
    }
  }
  return problems;
}

}  // namespace gsim
