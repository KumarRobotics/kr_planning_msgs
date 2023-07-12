#ifndef MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_
#define MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <kr_planning_msgs/SplineTrajectory.h>
#include <kr_planning_rviz_plugins/data_type.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>
namespace kr {
class SplineTrajectoryVisual {
 public:
  SplineTrajectoryVisual(Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node);

  virtual ~SplineTrajectoryVisual();

  void setMessage(const kr_planning_msgs::SplineTrajectory& msg);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setNum(int n);
  void setPosColor(float r, float g, float b, float a);
  void setVelColor(float r, float g, float b, float a);
  void setAccColor(float r, float g, float b, float a);
  void setPosScale(float s);
  void setVelScale(float s);
  void setAccScale(float s);
  void setVelVis(bool vis);
  void setAccVis(bool vis);
  std::vector<Eigen::VectorXd> sample(
      const kr_planning_msgs::SplineTrajectory& msg, int N, int deriv_num);
  static Eigen::VectorXd evaluate(const kr_planning_msgs::SplineTrajectory& msg,
                           double t,
                           uint deriv_num);
  static std::vector<float> differentiate(const std::vector<float>& p, float segment_time);

 private:
  std::vector<std::unique_ptr<rviz::BillboardLine>> poss_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> vels_;
  std::vector<std::unique_ptr<rviz::BillboardLine>> accs_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  int num_;
  bool vel_vis_;
  bool acc_vis_;
  bool jrk_vis_;
};
}  // namespace kr

#endif  // MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_
