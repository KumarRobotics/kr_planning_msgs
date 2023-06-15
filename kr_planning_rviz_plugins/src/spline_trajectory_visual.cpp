// Copyright 2016 Michael Watterson
// NOTE: The parameterization for the polynomial expected is s \in [0,1]
// Time duration dt is used to evaluate polynomial p(t/dt) for t \in [0,dt]

// #include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <kr_planning_rviz_plugins/spline_trajectory_visual.h>

#include <vector>

namespace kr {
SplineTrajectoryVisual::SplineTrajectoryVisual(
    Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

SplineTrajectoryVisual::~SplineTrajectoryVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void SplineTrajectoryVisual::setMessage(
    const kr_planning_msgs::SplineTrajectory& msg) {
  poss_.clear();
  vels_.clear();
  accs_.clear();

  if (num_ < 2) return;

  poss_.resize(num_ - 1);
  if (vel_vis_) vels_.resize(num_);
  if (acc_vis_) accs_.resize(num_);

  double theta = M_PI / 2;
  Mat3f R;
  R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

  const auto waypoints_pos = sample(msg, num_ - 1, 0);
  const auto waypoints_vel = sample(msg, num_ - 1, 1);
  const auto waypoints_acc = sample(msg, num_ - 1, 2);

  for (unsigned int i = 0; i < waypoints_pos.size(); i++) {
    const auto p1_pos = waypoints_pos[i];
    const auto p1_vel = waypoints_vel[i];
    const auto p1_acc = waypoints_acc[i];
    const Ogre::Vector3 pos1(p1_pos(0), p1_pos(1), p1_pos(2));

    if (i < waypoints_pos.size() - 1) {
      const auto p2 = waypoints_pos[i + 1];
      const Ogre::Vector3 pos2(p2(0), p2(1), p2(2));
      poss_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
      poss_[i]->addPoint(pos1);
      poss_[i]->addPoint(pos2);
    }

    if (vel_vis_) {
      const Vec3f p3 = p1_pos + R * p1_vel;
      const Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      vels_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
      vels_[i]->addPoint(pos1);
      vels_[i]->addPoint(pos3);
    }

    if (acc_vis_) {
      const Vec3f p3 = p1_pos + R * p1_acc;
      const Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      accs_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
      accs_[i]->addPoint(pos1);
      accs_[i]->addPoint(pos3);
    }
  }
}

void SplineTrajectoryVisual::setNum(int n) { num_ = n; }

void SplineTrajectoryVisual::setVelVis(bool vis) { vel_vis_ = vis; }

void SplineTrajectoryVisual::setAccVis(bool vis) { acc_vis_ = vis; }

void SplineTrajectoryVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void SplineTrajectoryVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void SplineTrajectoryVisual::setPosColor(float r, float g, float b, float a) {
  for (auto& it : poss_) it->setColor(r, g, b, a);
}

void SplineTrajectoryVisual::setVelColor(float r, float g, float b, float a) {
  for (auto& it : vels_) it->setColor(r, g, b, a);
}

void SplineTrajectoryVisual::setAccColor(float r, float g, float b, float a) {
  for (auto& it : accs_) it->setColor(r, g, b, a);
}

void SplineTrajectoryVisual::setPosScale(float s) {
  for (auto& it : poss_) it->setLineWidth(s);
}

void SplineTrajectoryVisual::setVelScale(float s) {
  for (auto& it : vels_) it->setLineWidth(s);
}

void SplineTrajectoryVisual::setAccScale(float s) {
  for (auto& it : accs_) it->setLineWidth(s);
}

std::vector<Eigen::VectorXd> SplineTrajectoryVisual::sample(
    const kr_planning_msgs::SplineTrajectory& msg, int N, int deriv_num) {
  std::vector<Eigen::VectorXd> ps(N + 1);
  double total_time = msg.data.front().t_total;
  double dt = total_time / N;
  for (int i = 0; i <= N; i++) ps.at(i) = evaluate(msg, i * dt, deriv_num);

  return ps;
}

std::vector<float> SplineTrajectoryVisual::differentiate(
    const std::vector<float>& p) const {
  if (p.size() < 2) return std::vector<float>();
  std::vector<float> v;
  for (int i = 1; i < p.size(); i++) {
    v.push_back(p[i] * static_cast<float>(i));
  }
  return v;
}

Eigen::VectorXd SplineTrajectoryVisual::evaluate(
    const kr_planning_msgs::SplineTrajectory& msg,
    double t,
    uint deriv_num) const {
  Eigen::VectorXd result(msg.dimensions);

  for (int dim = 0; dim < msg.dimensions; dim++) {
    auto spline = msg.data[dim];
    double dt = 0;
    for (auto poly : spline.segs) {
      auto poly_coeffs = poly.coeffs;
      for (int d = 0; d < deriv_num; d++) {
        poly_coeffs = differentiate(poly_coeffs);
      }
      result(dim) = poly_coeffs[0];

      if (t < dt + poly.dt || poly == spline.segs.back()) {
        for (int j = 1; j < poly_coeffs.size(); j++) {
          result(dim) += poly_coeffs[j] * std::pow((t - dt) / poly.dt, j);
        }
        break;
      }
      dt += poly.dt;
    }
  }
  return result;
}

}  // namespace kr