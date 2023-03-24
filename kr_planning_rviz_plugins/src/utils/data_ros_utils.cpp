#include <kr_planning_rviz_plugins/data_ros_utils.h>
namespace kr {
vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud& cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

vec_Vec3f cloud_to_vec_filter(const sensor_msgs::PointCloud& cloud,
                              const double eps) {
  vec_Vec3f pts;
  pts.reserve(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    if (pow(cloud.points[i].x, 2.0) + pow(cloud.points[i].y, 2.0) +
            pow(cloud.points[i].z, 2.0) >
        eps) {
      Vec3f newPt;
      newPt(0) = cloud.points[i].x;
      newPt(1) = cloud.points[i].y;
      newPt(2) = cloud.points[i].z;
      pts.push_back(newPt);
    }
  }

  return pts;
}

vec_Vec3f ros_to_path(const kr_planning_msgs::Path& msg) {
  vec_Vec3f path;
  for (const auto& it : msg.waypoints) {
    path.push_back(Vec3f(it.x, it.y, it.z));
  }
  return path;
}

 double p(double t, std::vector<double> c) {
  return c.at(0) / 120 * std::pow(t, 5) + c.at(1) / 24 * std::pow(t, 4) +
         c.at(2) / 6 * std::pow(t, 3) + c.at(3) / 2 * t * t + c.at(4) * t +
         c.at(5);
}

 Eigen::Vector3d evaluate_position(
    const kr_planning_msgs::Trajectory& msg, double t) {
  Eigen::Vector3d result(3);

  double dt = 0;
  for (const auto& primitive : msg.primitives) {
    if (t < dt + primitive.t || primitive == msg.primitives.back()) {
      result(0) = p(t - dt, primitive.cx);
      result(1) = p(t - dt , primitive.cy);
      result(2) = p(t - dt, primitive.cz);
      break;
    }
    dt += primitive.t;
  }
  return result;
}

 std::vector<Eigen::Vector3d> sample_position(
    const kr_planning_msgs::Trajectory& msg, int N) {
  std::vector<Eigen::Vector3d> ps(N + 1);
  double total_time = 0;
  for (const auto& primitive : msg.primitives) {
    total_time += primitive.t;
  }
  double dt = total_time / N;
  for (int i = 0; i <= N; i++) ps.at(i) = evaluate_position(msg, i * dt);

  return ps;
}

}  // namespace kr