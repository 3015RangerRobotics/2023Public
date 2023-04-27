package frc.lib.vision.jetson;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.*;
import java.util.Optional;

public class AprilTagDetection {
  private final AprilTag tag;
  private final double timestampSeconds;

  public AprilTagDetection(AprilTag tag, double timestampSeconds) {
    this.tag = tag;
    this.timestampSeconds = timestampSeconds;
  }

  public int getTagID() {
    return this.tag.ID;
  }

  public double getTimestampSeconds() {
    return this.timestampSeconds;
  }

  public Transform3d getCameraToTag() {
    return new Transform3d(this.tag.pose.getTranslation(), this.tag.pose.getRotation());
  }

  public Transform3d getRobotToTag(Transform3d cameraToRobot) {
    Transform3d robotToCamera = cameraToRobot.inverse();
    return robotToCamera.plus(getCameraToTag());
  }

  /**
   * Get the estimated pose of the tag on the field
   *
   * @param cameraToRobot Transform from the camera to the robot
   * @param robotPose Current robot pose on the field
   * @return Pose of the tag on the field
   */
  public Pose3d getEstimatedTagPose(Transform3d cameraToRobot, Pose2d robotPose) {
    Transform3d robotToTag = getRobotToTag(cameraToRobot);
    Transform3d fieldToRobot =
        new Transform3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), 0),
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d fieldToTag = fieldToRobot.plus(robotToTag);
    return new Pose3d(fieldToTag.getTranslation(), fieldToTag.getRotation());
  }

  public Optional<Pose3d> getEstimatedRobotPose(
      AprilTagFieldLayout layout, Transform3d robotToCamera) {
    Optional<Pose3d> fieldToTarget = layout.getTagPose(getTagID());
    return fieldToTarget.map(
        pose3d -> ComputerVisionUtil.objectToRobotPose(pose3d, getCameraToTag(), robotToCamera));
  }
}
