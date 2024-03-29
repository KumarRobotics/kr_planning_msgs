#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <kr_planning_msgs/Path.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/visualization_manager.h>

#include "path_visual.h"

namespace kr {
class PathDisplay : public rviz::MessageFilterDisplay<kr_planning_msgs::Path> {
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
  void processMessage(const kr_planning_msgs::Path::ConstPtr& msg);
  void visualizeMessage();

  std::shared_ptr<PathVisual> visual_;

  rviz::ColorProperty* line_color_property_;
  rviz::ColorProperty* node_color_property_;
  rviz::FloatProperty* line_scale_property_;
  rviz::FloatProperty* node_scale_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_Vec3f path_;
};
}  // namespace kr
