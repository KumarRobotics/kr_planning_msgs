#pragma once

#include <kr_planning_msgs/VoxelMap.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <kr_planning_rviz_plugins/map_util.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>

#include <cmath>
#include <memory>
#include <vector>

#include "./bound_visual.h"
#include "./mesh_visual.h"

namespace kr_planning_rviz_plugins {
class MapDisplay
    : public rviz::MessageFilterDisplay<kr_planning_msgs::VoxelMap> {
  Q_OBJECT
 public:
  MapDisplay();
  ~MapDisplay();

  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

 private Q_SLOTS:
  void updateState();
  void updateBoundScale();
  void updateMeshColorAndAlpha();
  void updateMeshHeight();

 protected:
  void setMap(std::shared_ptr<VoxelMapUtil>& map_util,
              const kr_planning_msgs::VoxelMap& msg);
  void getMap(std::shared_ptr<VoxelMapUtil>& map_util,
              kr_planning_msgs::VoxelMap& map);

  void onInitialize();

  void processMessage(const kr_planning_msgs::VoxelMapConstPtr& map);
  void visualizeMessage(int state);
  void visualizeMesh(const vec_Vec3f& pts, double res);
  vec_E<vec_Vec3f> getBound();

  rviz::EnumProperty* state_property_;
  rviz::FloatProperty* bound_scale_property_;
  rviz::FloatProperty* mesh_height_property_;
  rviz::ColorProperty* mesh_color_property_;
  rviz::FloatProperty* mesh_alpha_property_;

  rviz::PointCloudCommon* point_cloud_common_;
  std::shared_ptr<BoundVisual> visual_;
  std::vector<std::shared_ptr<MeshVisual>> visuals_mesh_;
  float mesh_height_;

  std::shared_ptr<VoxelMapUtil> map_util_;
  std::shared_ptr<std_msgs::Header> header_ptr_;

  Ogre::Quaternion orientation_;
  Ogre::Vector3 position_;
};

}  // namespace kr_planning_rviz_plugins
