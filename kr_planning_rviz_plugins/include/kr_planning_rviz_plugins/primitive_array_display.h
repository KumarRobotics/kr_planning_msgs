#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/msg/primitive_array.hpp>
// #include <rviz_common/frame_manager_iface.hpp>
// #include <rviz_common/load_resource.hpp>
// #include <rviz_common/message_filter_display.hpp>
// #include <rviz_common/properties/bool_property.hpp>
// #include <rviz_common/properties/color_property.hpp>
// #include <rviz_common/properties/float_property.hpp>
// #include <rviz_common/properties/int_property.hpp>
// #include <rviz_common/visualization_manager.hpp>

#include <rviz_common/frame_manager_iface.hpp>  // Replaces rviz_common/frame_manager_iface.hpp
#include <rviz_common/load_resource.hpp>       // Replaces rviz_common/load_resource.hpp
#include <rviz_common/message_filter_display.hpp> // Replaces rviz_common/message_filter_display.hpp
#include <rviz_common/properties/bool_property.hpp> // Replaces rviz_common/properties/bool_property.hpp
#include <rviz_common/properties/color_property.hpp> // Replaces rviz_common/properties/color_property.hpp
#include <rviz_common/properties/float_property.hpp> // Replaces rviz_common/properties/float_property.hpp
#include <rviz_common/properties/int_property.hpp>  // Replaces rviz_common/properties/int_property.hpp
#include <rviz_common/visualization_manager.hpp> // Replaces rviz_common/visualization_manager.hpp

#include "primitive_visual.h"

namespace kr {
class PrimitiveArrayDisplay
    : public rviz_common::MessageFilterDisplay<kr_planning_msgs::msg::PrimitiveArray> {
  Q_OBJECT
 public:
  PrimitiveArrayDisplay();
  virtual ~PrimitiveArrayDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updatePosColorAndAlpha();
  void updateVelColorAndAlpha();
  void updateAccColorAndAlpha();
  void updateJrkColorAndAlpha();
  void updateYawColorAndAlpha();
  void updatePosScale();
  void updateVelScale();
  void updateAccScale();
  void updateJrkScale();
  void updateYawScale();
  void updateYawTriangleScale();
  void updateYawTriangleAngle();
  void updateVelVis();
  void updateAccVis();
  void updateJrkVis();
  void updateYawVis();
  void updateNum();
  void updateYawNum();

 private:
  void processMessage(kr_planning_msgs::msg::PrimitiveArray::ConstSharedPtr msg);
  void visualizeMessage();

  std::shared_ptr<PrimitiveVisual> visual_;

  rviz_common::properties::ColorProperty* pos_color_property_;
  rviz_common::properties::ColorProperty* vel_color_property_;
  rviz_common::properties::ColorProperty* acc_color_property_;
  rviz_common::properties::ColorProperty* jrk_color_property_;
  rviz_common::properties::ColorProperty* yaw_color_property_;
  rviz_common::properties::FloatProperty* pos_scale_property_;
  rviz_common::properties::FloatProperty* vel_scale_property_;
  rviz_common::properties::FloatProperty* acc_scale_property_;
  rviz_common::properties::FloatProperty* jrk_scale_property_;
  rviz_common::properties::FloatProperty* yaw_scale_property_;
  rviz_common::properties::FloatProperty* yaw_triangle_scale_property_;
  rviz_common::properties::FloatProperty* yaw_triangle_angle_property_;
  rviz_common::properties::BoolProperty* vel_vis_property_;
  rviz_common::properties::BoolProperty* acc_vis_property_;
  rviz_common::properties::BoolProperty* jrk_vis_property_;
  rviz_common::properties::BoolProperty* yaw_vis_property_;
  rviz_common::properties::IntProperty* num_property_;
  rviz_common::properties::IntProperty* yaw_num_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  kr_planning_msgs::msg::PrimitiveArray prs_msg_;
};
}  // namespace kr
