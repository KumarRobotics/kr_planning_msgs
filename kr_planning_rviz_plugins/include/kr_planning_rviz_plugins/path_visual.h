#ifndef PATH_VISUAL_H
#define PATH_VISUAL_H

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <kr_planning_msgs/Path.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>

namespace kr {
class PathVisual {
 public:
  PathVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  virtual ~PathVisual();

  void setMessage(const vec_Vec3f& path);
  void addMessage(const vec_Vec3f& path);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setLineColor(float r, float g, float b, float a);
  void setNodeColor(float r, float g, float b, float a);
  void setLineScale(float s);
  void setNodeScale(float s);

 private:
  std::vector<std::unique_ptr<rviz::BillboardLine>> lines_;
  std::vector<std::unique_ptr<rviz::Shape>> nodes_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};
}  // namespace kr

#endif
