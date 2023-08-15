/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */
// TODO(Laura) move somewhere that makes more sense

#include <kr_planning_msgs_utils/image_loader.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <kr_planning_rviz_plugins/map_util.h>
#include <libgen.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <stdlib.h>

#include <filesystem>
#include <fstream>
std::string mapfpath;
double origin[3];
int negate;
double occ_th, free_th;
double res;
MapMode mode = TRINARY;
std::string frame_id;
bool service;
bool increment_flag;
std::filesystem::directory_iterator iterator;
ros::Publisher cloud_pub;

sensor_msgs::PointCloud2 createCloud(std::string path) {
  std::cout << path << std::endl;
  kr_planning_msgs::VoxelMap map_resp;
  try {
    map_server::loadMapFromFile(
        map_resp, path.c_str(), res, negate, occ_th, free_th, origin, mode);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    exit(-1);
  }
  // To make sure get a consistent time in simulation
  map_resp.header.frame_id = frame_id;
  map_resp.header.stamp = ros::Time::now();
  ROS_INFO("Read a %.1f X %.1f map @ %.3lf m/cell",
           map_resp.dim.x,
           map_resp.dim.y,
           map_resp.resolution);
  // Latched publisher for data
  // map_pub.publish(map_resp);

  kr::VoxelMapUtil map_util;
  Vec3f ori(map_resp.origin.x, map_resp.origin.y, map_resp.origin.z);
  Vec3i dim(map_resp.dim.x, map_resp.dim.y, map_resp.dim.z);
  std::vector<signed char> map = map_resp.data;
  map_util.setMap(ori, dim, map, res);
  auto cloud = kr::vec_to_cloud(map_util.getCloud());
  cloud.header = map_resp.header;
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
  return cloud2;
}

bool image_to_map_server(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& res) {
  if (iterator != std::filesystem::directory_iterator())
    ++iterator;
  else {
    ROS_INFO("No more files in directory");
    return false;
  }
  auto cloud2 = createCloud(iterator->path());
  for (uint i = 0; i < 5; i++) {
    cloud_pub.publish(cloud2);
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "create_map", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  ros::Rate loop_rate(0.2);
  // Yifei: No longer publishing voxel from this node, moved to mapper node
  //  ros::Publisher map_pub =
  //      nh.advertise<kr_planning_msgs::VoxelMap>("voxel_map", 1, true);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);

  nh.param("resolution", res, 0.1);
  nh.param("frame_id", frame_id, std::string("map"));
  nh.param("negate", negate, 0);
  nh.param("occ_th", occ_th, 0.65);
  nh.param("free_th", free_th, 0.2);

  nh.param("origin_x", origin[0], 0.0);
  nh.param("origin_y", origin[1], 0.0);
  nh.param("origin_z", origin[2], 0.0);

  nh.param("file_path", mapfpath, std::string(""));
  nh.param("service", service, false);
  iterator = std::filesystem::directory_iterator(mapfpath);

  while (ros::ok() && cloud_pub.getNumSubscribers() == 0) {
    ROS_INFO("Waiting for subscriber to connect to %s",
             cloud_pub.getTopic().c_str());
    ros::Duration(0.1).sleep();
  }
  if (!service) {
    while (ros::ok()) {
      // iterate through the files in mapfpath and load them and populate
      // map_resp with data
      for (auto& p : iterator) {
        auto cloud2 = createCloud(p.path());
        // publish the same map twice so decay is not an issue
        for (uint i = 0; i < 2; i++) {
          cloud_pub.publish(cloud2);
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
  }

  else {
    ros::ServiceServer service =
        nh.advertiseService("/image_to_map", image_to_map_server);
    ros::spin();
  }

  return 0;
}
