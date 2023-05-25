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

double v(double t, std::vector<double> c) {
  return c.at(0) / 24 * std::pow(t, 4) + c.at(1) / 6 * std::pow(t, 3) +
         c.at(2) / 2 * t * t + c.at(3) * t + c.at(4);
}

double a(double t, std::vector<double> c) {
  return c.at(0) / 6 * std::pow(t, 3) + c.at(1) / 2 * t * t + c.at(2) * t +
         c.at(3);
}

double j(double t, std::vector<double> c) {
  return c.at(0) / 2 * std::pow(t, 2) + c.at(1) * t + c.at(2);
}

double evaluator(double t, std::vector<double> c, int deriv_num) {
  switch (deriv_num) {
    case 0:
      return p(t, c);
    case 1:
      return v(t, c);
    case 2:
      return a(t, c);
    case 3:
      return j(t, c);
    default:
      return 0;
  }
}
Eigen::Vector3d evaluate(const kr_planning_msgs::Trajectory& msg,
                         double t,
                         int deriv_num) {
  Eigen::Vector3d result(3);

  double dt = 0;
  for (const auto& primitive : msg.primitives) {
    if (t < dt + primitive.t || primitive == msg.primitives.back()) {
      result(0) = evaluator(t - dt, primitive.cx, deriv_num);
      result(1) = evaluator(t - dt, primitive.cy, deriv_num);
      result(2) = evaluator(t - dt, primitive.cz, deriv_num);
      break;
    }
    dt += primitive.t;
  }
  return result;
}

std::vector<Eigen::Vector3d> sample(const kr_planning_msgs::Trajectory& msg,
                                    int N,
                                    int deriv_num) {
  std::vector<Eigen::Vector3d> ps(N + 1);
  double total_time = 0;
  for (const auto& primitive : msg.primitives) {
    total_time += primitive.t;
  }
  double dt = total_time / N;
  for (int i = 0; i <= N; i++) ps.at(i) = evaluate(msg, i * dt, deriv_num);

  return ps;
}

}  // namespace kr