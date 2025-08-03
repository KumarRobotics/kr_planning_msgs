
#ifndef MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
#define MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/msg/spline_trajectory.hpp>
#include <kr_planning_rviz_plugins/spline_trajectory_visual.h>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/visualization_manager.hpp>

namespace kr {
class SplineTrajectoryDisplay
    : public rviz_common::MessageFilterDisplay<kr_planning_msgs::msg::SplineTrajectory> {
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
  void processMessage(kr_planning_msgs::msg::SplineTrajectory::ConstSharedPtr msg);
  void visualizeMessage();

  std::shared_ptr<SplineTrajectoryVisual> visual_;

  rviz_common::properties::ColorProperty* pos_color_property_;
  rviz_common::properties::ColorProperty* vel_color_property_;
  rviz_common::properties::ColorProperty* acc_color_property_;
  rviz_common::properties::FloatProperty* pos_scale_property_;
  rviz_common::properties::FloatProperty* vel_scale_property_;
  rviz_common::properties::FloatProperty* acc_scale_property_;
  rviz_common::properties::BoolProperty* vel_vis_property_;
  rviz_common::properties::BoolProperty* acc_vis_property_;
  rviz_common::properties::IntProperty* num_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  kr_planning_msgs::msg::SplineTrajectory trajectory_;
};
}  // namespace kr

#endif  // MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_DISPLAY_H_
