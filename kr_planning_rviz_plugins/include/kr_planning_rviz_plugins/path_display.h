#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/msg/path.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/visualization_manager.hpp>



#include "path_visual.h"

namespace kr {
class PathDisplay : public rviz_common::MessageFilterDisplay<kr_planning_msgs::msg::Path> {
  Q_OBJECT

 public:
  PathDisplay();
  virtual ~PathDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private Q_SLOTS:
  void updateLineColorAndAlpha();
  void updateNodeColorAndAlpha();
  void updateLineScale();
  void updateNodeScale();

 private:
  void processMessage(kr_planning_msgs::msg::Path::ConstSharedPtr msg);
  void visualizeMessage();

  std::shared_ptr<PathVisual> visual_;

  rviz_common::properties::ColorProperty* line_color_property_;
  rviz_common::properties::ColorProperty* node_color_property_;
  rviz_common::properties::FloatProperty* line_scale_property_;
  rviz_common::properties::FloatProperty* node_scale_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_Vec3f path_;
};
}  // namespace kr
