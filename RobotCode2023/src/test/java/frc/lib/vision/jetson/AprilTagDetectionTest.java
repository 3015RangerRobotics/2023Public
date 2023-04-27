package frc.lib.vision.jetson;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class AprilTagDetectionTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testAprilTagDetection() {
    AprilTagDetection detection =
        new AprilTagDetection(
            new AprilTag(109, new Pose3d(0.5, -0.5, 0.0, new Rotation3d(0.5, 1.0, 1.5))), 5.73);

    assertEquals(109, detection.getTagID());
    assertEquals(5.73, detection.getTimestampSeconds(), DELTA);
    assertEquals(
        new Transform3d(new Translation3d(0.5, -0.5, 0.0), new Rotation3d(0.5, 1.0, 1.5)),
        detection.getCameraToTag());
  }

  @Test
  public void testRobotToTag() {
    AprilTagDetection detection =
        new AprilTagDetection(
            new AprilTag(109, new Pose3d(2.0, -0.5, 0.0, new Rotation3d(0, -0.5, 0))), 5.73);
    Transform3d cameraToRobot =
        new Transform3d(new Translation3d(-0.25, 0.25, 0.0), new Rotation3d(0.0, -0.5, 0.0));

    Transform3d robotToTag = detection.getRobotToTag(cameraToRobot);

    assertEquals(1.97, robotToTag.getTranslation().getX(), DELTA);
    assertEquals(-0.75, robotToTag.getTranslation().getY(), DELTA);
    assertEquals(-1.08, robotToTag.getTranslation().getZ(), DELTA);
    assertEquals(0, robotToTag.getRotation().getX(), DELTA);
    assertEquals(0, robotToTag.getRotation().getY(), DELTA);
    assertEquals(0, robotToTag.getRotation().getZ(), DELTA);
  }

  @Test
  public void testEstimatedTagPose() {
    AprilTagDetection detection =
        new AprilTagDetection(
            new AprilTag(109, new Pose3d(2.0, -0.5, 0.0, new Rotation3d(0, -0.5, 0))), 5.73);
    Transform3d cameraToRobot =
        new Transform3d(new Translation3d(-0.25, 0.25, 0.0), new Rotation3d(0.0, -0.5, 0.0));
    Pose2d robotPose = new Pose2d(0.5, 0.5, new Rotation2d(0.5));

    Pose3d fieldRelativePose = detection.getEstimatedTagPose(cameraToRobot, robotPose);

    assertEquals(2.59, fieldRelativePose.getTranslation().getX(), DELTA);
    assertEquals(0.79, fieldRelativePose.getTranslation().getY(), DELTA);
    assertEquals(-1.08, fieldRelativePose.getTranslation().getZ(), DELTA);
    assertEquals(0, fieldRelativePose.getRotation().getX(), DELTA);
    assertEquals(0, fieldRelativePose.getRotation().getY(), DELTA);
    assertEquals(0.5, fieldRelativePose.getRotation().getZ(), DELTA);
  }

  @Test
  public void testEstimatedRobotPose() {
    AprilTagDetection detection1 =
        new AprilTagDetection(
            new AprilTag(0, new Pose3d(1.0, 0.0, 1.0, new Rotation3d(0.0, 0.5, 0.5))), 0);

    AprilTagFieldLayout layout =
        new AprilTagFieldLayout(
            List.of(new AprilTag(0, new Pose3d(1.5, 4.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))),
            10,
            10);
    Transform3d cameraToRobot =
        new Transform3d(new Translation3d(-0.25, 0.25, 0.0), new Rotation3d(0.0, -0.5, 0.0));

    Optional<Pose3d> estimated = detection1.getEstimatedRobotPose(layout, cameraToRobot);

    assertTrue(estimated.isPresent());

    Pose3d estimatedPose = estimated.get();

    assertEquals(1.33, estimatedPose.getX(), DELTA);
    assertEquals(4.15, estimatedPose.getY(), DELTA);
    assertEquals(-1.37, estimatedPose.getZ(), DELTA);
    assertEquals(0.23, estimatedPose.getRotation().getX(), DELTA);
    assertEquals(0.05, estimatedPose.getRotation().getY(), DELTA);
    assertEquals(-0.43, estimatedPose.getRotation().getZ(), DELTA);
  }
}
