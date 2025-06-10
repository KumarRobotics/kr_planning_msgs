#pragma once

#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <kr_planning_rviz_plugins/map_util.h>

// #include <rviz/default_plugin/point_cloud_common.h>
// #include <rviz/display_context.h>
// #include <rviz_common/frame_manager_iface.hpp>
// #include <rviz_common/message_filter_display.hpp>
// #include <rviz/ogre_helpers/point_cloud.h>
// #include <rviz_common/properties/enum_property.hpp>
// #include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>  // Note: FrameManager is now an interface
#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>  // Note: Ogre helpers moved to rviz_rendering
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <cmath>
#include <memory>
#include <vector>


#include "./bound_visual.h"
#include "./mesh_visual.h"

namespace kr {
class MapDisplay
    : public rviz_common::MessageFilterDisplay<kr_planning_msgs::msg::VoxelMap> {
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
              const kr_planning_msgs::msg::VoxelMap& msg);
  void getMap(std::shared_ptr<VoxelMapUtil>& map_util,
              kr_planning_msgs::msg::VoxelMap& map);

  void onInitialize();

  void processMessage(kr_planning_msgs::msg::VoxelMap::ConstSharedPtr map);
  void visualizeMessage(int state);
  void visualizeMesh(const vec_Vec3f& pts, double res);
  vec_E<vec_Vec3f> getBound();

  rviz_common::properties::EnumProperty* state_property_;
  rviz_common::properties::FloatProperty* bound_scale_property_;
  rviz_common::properties::FloatProperty* mesh_height_property_;
  rviz_common::properties::ColorProperty* mesh_color_property_;
  rviz_common::properties::FloatProperty* mesh_alpha_property_;

  rviz_default_plugins::PointCloudCommon* point_cloud_common_;
  std::shared_ptr<BoundVisual> visual_;
  std::vector<std::shared_ptr<MeshVisual>> visuals_mesh_;
  float mesh_height_;

  std::shared_ptr<VoxelMapUtil> map_util_;
  std::shared_ptr<std_msgs::msg::Header> header_ptr_;

  Ogre::Quaternion orientation_;
  Ogre::Vector3 position_;
};

}  // namespace kr
