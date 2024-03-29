#ifndef MESH_VISUAL_H
#define MESH_VISUAL_H

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <kr_planning_rviz_plugins/data_type.h>
#include <rviz/ogre_helpers/mesh_shape.h>

namespace kr {
class MeshVisual {
 public:
  MeshVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  virtual ~MeshVisual();

  void setMessage(const vec_E<vec_Vec3f>& vss);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColor(float r, float g, float b, float a);

 private:
  std::unique_ptr<rviz::MeshShape> obj_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};
}  // namespace kr

#endif
