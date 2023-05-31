#include <kr_planning_rviz_plugins/spline_trajectory_display.h>

namespace kr {
SplineTrajectoryDisplay::SplineTrajectoryDisplay() {
  num_property_ =
      new rviz::IntProperty("Num of samples",
                            100,
                            "Number of samples of trajectory to display.",
                            this,
                            SLOT(updateNum()));

  pos_color_property_ = new rviz::ColorProperty("PosColor",
                                                QColor(204, 51, 204),
                                                "Color to draw the Pos.",
                                                this,
                                                SLOT(updatePosColorAndAlpha()));
  vel_color_property_ = new rviz::ColorProperty("VelColor",
                                                QColor(85, 85, 255),
                                                "Color to draw the Vel.",
                                                this,
                                                SLOT(updateVelColorAndAlpha()));
  acc_color_property_ = new rviz::ColorProperty("AccColor",
                                                QColor(10, 200, 55),
                                                "Color to draw the Acc.",
                                                this,
                                                SLOT(updateAccColorAndAlpha()));
  pos_scale_property_ = new rviz::FloatProperty("PosScale",
                                                0.1,
                                                "0.1 is the default value.",
                                                this,
                                                SLOT(updatePosScale()));
  vel_scale_property_ = new rviz::FloatProperty("VelScale",
                                                0.02,
                                                "0.02 is the default value.",
                                                this,
                                                SLOT(updateVelScale()));
  acc_scale_property_ = new rviz::FloatProperty("AccScale",
                                                0.02,
                                                "0.02 is the default value.",
                                                this,
                                                SLOT(updateAccScale()));

  vel_vis_property_ = new rviz::BoolProperty(
      "VelVis", 0, "Visualize Vel?", this, SLOT(updateVelVis()));
  acc_vis_property_ = new rviz::BoolProperty(
      "AccVis", 0, "Visualize Acc?", this, SLOT(updateAccVis()));
}

void SplineTrajectoryDisplay::onInitialize() { MFDClass::onInitialize(); }

SplineTrajectoryDisplay::~SplineTrajectoryDisplay() {}

void SplineTrajectoryDisplay::reset() {
  MFDClass::reset();
  visual_ = nullptr;
}

void SplineTrajectoryDisplay::updateVelVis() { visualizeMessage(); }

void SplineTrajectoryDisplay::updateAccVis() { visualizeMessage(); }

void SplineTrajectoryDisplay::updateJrkVis() { visualizeMessage(); }

void SplineTrajectoryDisplay::updateYawVis() { visualizeMessage(); }

void SplineTrajectoryDisplay::updatePosColorAndAlpha() {
  Ogre::ColourValue color = pos_color_property_->getOgreColor();
  if (visual_) visual_->setPosColor(color.r, color.g, color.b, 1);
}

void SplineTrajectoryDisplay::updateVelColorAndAlpha() {
  Ogre::ColourValue color = vel_color_property_->getOgreColor();
  if (visual_) visual_->setVelColor(color.r, color.g, color.b, 1);
}

void SplineTrajectoryDisplay::updateAccColorAndAlpha() {
  Ogre::ColourValue color = acc_color_property_->getOgreColor();
  if (visual_) visual_->setAccColor(color.r, color.g, color.b, 1);
}

void SplineTrajectoryDisplay::updatePosScale() {
  float s = pos_scale_property_->getFloat();
  if (visual_) visual_->setPosScale(s);
}

void SplineTrajectoryDisplay::updateVelScale() {
  float s = vel_scale_property_->getFloat();
  if (visual_) visual_->setVelScale(s);
}

void SplineTrajectoryDisplay::updateAccScale() {
  float s = acc_scale_property_->getFloat();
  if (visual_) visual_->setAccScale(s);
}

void SplineTrajectoryDisplay::updateNum() { visualizeMessage(); }

void SplineTrajectoryDisplay::processMessage(
    const kr_planning_msgs::SplineTrajectory::ConstPtr& msg) {
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  trajectory_ = *msg;

  visualizeMessage();
}

void SplineTrajectoryDisplay::visualizeMessage() {
  visual_.reset(
      new SplineTrajectoryVisual(context_->getSceneManager(), scene_node_));

  if (trajectory_.data.empty() || !pos_color_property_ ||
      !vel_color_property_ || !acc_color_property_ || !pos_scale_property_ ||
      !vel_scale_property_ || !acc_scale_property_ || !vel_vis_property_ ||
      !acc_vis_property_ || !num_property_)
    return;

  float n = num_property_->getInt();
  visual_->setNum(n);

  bool vel_vis = vel_vis_property_->getBool();
  visual_->setVelVis(vel_vis);

  bool acc_vis = acc_vis_property_->getBool();
  visual_->setAccVis(acc_vis);

  visual_->setMessage(trajectory_);

  visual_->setFramePosition(position_);
  visual_->setFrameOrientation(orientation_);

  float pos_scale = pos_scale_property_->getFloat();
  visual_->setPosScale(pos_scale);
  float vel_scale = vel_scale_property_->getFloat();
  visual_->setVelScale(vel_scale);
  float acc_scale = acc_scale_property_->getFloat();
  visual_->setAccScale(acc_scale);

  Ogre::ColourValue pos_color = pos_color_property_->getOgreColor();
  visual_->setPosColor(pos_color.r, pos_color.g, pos_color.b, 1);
  Ogre::ColourValue vel_color = vel_color_property_->getOgreColor();
  visual_->setVelColor(vel_color.r, vel_color.g, vel_color.b, 1);
  Ogre::ColourValue acc_color = acc_color_property_->getOgreColor();
  visual_->setAccColor(acc_color.r, acc_color.g, acc_color.b, 1);
}
}  // namespace kr

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kr::SplineTrajectoryDisplay, rviz::Display)
