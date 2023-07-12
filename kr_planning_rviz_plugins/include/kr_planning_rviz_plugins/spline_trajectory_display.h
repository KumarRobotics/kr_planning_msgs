
#ifndef MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
#define MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/SplineTrajectory.h>
#include <kr_planning_rviz_plugins/spline_trajectory_visual.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

namespace kr {
class SplineTrajectoryDisplay
    : public rviz::MessageFilterDisplay<kr_planning_msgs::SplineTrajectory> {
  Q_OBJECT
 public:
  SplineTrajectoryDisplay();
  virtual ~SplineTrajectoryDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updatePosColorAndAlpha();
  void updateVelColorAndAlpha();
  void updateAccColorAndAlpha();
  void updatePosScale();
  void updateVelScale();
  void updateAccScale();
  void updateVelVis();
  void updateAccVis();
  void updateJrkVis();
  void updateYawVis();
  void updateNum();

 private:
  void processMessage(const kr_planning_msgs::SplineTrajectory::ConstPtr& msg);
  void visualizeMessage();

  std::shared_ptr<SplineTrajectoryVisual> visual_;

  rviz::ColorProperty* pos_color_property_;
  rviz::ColorProperty* vel_color_property_;
  rviz::ColorProperty* acc_color_property_;
  rviz::FloatProperty* pos_scale_property_;
  rviz::FloatProperty* vel_scale_property_;
  rviz::FloatProperty* acc_scale_property_;
  rviz::BoolProperty* vel_vis_property_;
  rviz::BoolProperty* acc_vis_property_;
  rviz::IntProperty* num_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  kr_planning_msgs::SplineTrajectory trajectory_;
};
}  // namespace kr

#endif  // MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
