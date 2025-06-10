#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/msg/path_array.hpp>
// #include <rviz_common/frame_manager_iface.hpp>
// #include <rviz_common/load_resource.hpp>
// #include <rviz_common/message_filter_display.hpp>
// #include <rviz_common/properties/color_property.hpp>
// #include <rviz_common/properties/enum_property.hpp>
// #include <rviz_common/properties/float_property.hpp>
// #include <rviz_common/properties/int_property.hpp>
// #include <rviz_common/visualization_manager.hpp>

#include <rviz_common/frame_manager_iface.hpp>  // Replaces rviz_common/frame_manager_iface.hpp
#include <rviz_common/load_resource.hpp>       // Replaces rviz_common/load_resource.hpp
#include <rviz_common/message_filter_display.hpp> // Replaces rviz_common/message_filter_display.hpp
#include <rviz_common/properties/bool_property.hpp> // Replaces rviz_common/properties/bool_property.hpp
#include <rviz_common/properties/color_property.hpp> // Replaces rviz_common/properties/color_property.hpp
#include <rviz_common/properties/enum_property.hpp> // Replaces rviz_common/properties/enum_property.hpp
#include <rviz_common/properties/float_property.hpp> // Replaces rviz_common/properties/float_property.hpp
#include <rviz_common/properties/int_property.hpp>  // Replaces rviz_common/properties/int_property.hpp
#include <rviz_common/visualization_manager.hpp> // Replaces rviz_common/visualization_manager.hpp

#include "path_visual.h"

namespace kr {
class PathArrayDisplay
    : public rviz_common::MessageFilterDisplay<kr_planning_msgs::msg::PathArray> {
  Q_OBJECT
 public:
  PathArrayDisplay();
  virtual ~PathArrayDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updateLineColorAndAlpha();
  void updateNodeColorAndAlpha();
  void updateLineScale();
  void updateNodeScale();
  void updateID();

 private:
  void processMessage(kr_planning_msgs::msg::PathArray::ConstSharedPtr msg);
  void visualizeMessage(int state);

  std::shared_ptr<PathVisual> visual_;

  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::ColorProperty* node_color_property_;
  rviz_common::properties::FloatProperty* line_scale_property_;
  rviz_common::properties::FloatProperty* node_scale_property_;
  rviz_common::properties::EnumProperty* id_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  kr_planning_msgs::msg::PathArray paths_;
};
}  // namespace kr
