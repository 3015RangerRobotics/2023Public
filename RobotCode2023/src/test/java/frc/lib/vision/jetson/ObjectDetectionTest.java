package frc.lib.vision.jetson;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import frc.lib.util.Vector3;
import org.junit.jupiter.api.Test;

public class ObjectDetectionTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testObjectDetection() {
    ObjectDetection detection1 =
        new ObjectDetection(
            "test",
            0,
            0.8,
            ObjectDetection.TrackingState.OFF,
            false,
            new Translation3d(),
            new Vector3(1, 2, 3),
            new Vector3(4, 5, 6),
            5.0);

    assertEquals("test", detection1.getClassName());
    assertEquals(0, detection1.getObjectID());
    assertEquals(0.8, detection1.getConfidence(), DELTA);
    assertEquals(ObjectDetection.TrackingState.OFF, detection1.getTrackingState());
    assertFalse(detection1.isMoving());
    assertEquals(new Translation3d(), detection1.getCameraToObject());
    assertEquals(new Vector3(0, 0, 0), detection1.getCameraRelativeVelocity());
    assertEquals(new Vector3(4, 5, 6), detection1.getDimensions());
    assertEquals(5.0, detection1.getTimestampSeconds(), DELTA);

    ObjectDetection detection2 =
        new ObjectDetection(
            "test2",
            1,
            0.6,
            ObjectDetection.TrackingState.OK,
            true,
            new Translation3d(1, 2, 3),
            new Vector3(1, 1, 1),
            new Vector3(2, 2, 2),
            8.1);

    assertEquals("test2", detection2.getClassName());
    assertEquals(1, detection2.getObjectID());
    assertEquals(0.6, detection2.getConfidence(), DELTA);
    assertEquals(ObjectDetection.TrackingState.OK, detection2.getTrackingState());
    assertTrue(detection2.isMoving());
    assertEquals(new Translation3d(1, 2, 3), detection2.getCameraToObject());
    assertEquals(new Vector3(1, 1, 1), detection2.getCameraRelativeVelocity());
    assertEquals(new Vector3(2, 2, 2), detection2.getDimensions());
    assertEquals(8.1, detection2.getTimestampSeconds(), DELTA);
  }

  @Test
  public void testTranslationTransforms() {
    ObjectDetection detection =
        new ObjectDetection(
            "test",
            0,
            0.8,
            ObjectDetection.TrackingState.OFF,
            false,
            new Translation3d(1.5, -0.5, -0.25),
            new Vector3(0, 0, 0),
            new Vector3(0, 0, 0),
            5.0);

    Translation3d cameraToObject = detection.getCameraToObject();
    assertEquals(1.5, cameraToObject.getX(), DELTA);
    assertEquals(-0.5, cameraToObject.getY(), DELTA);
    assertEquals(-0.25, cameraToObject.getZ(), DELTA);

    Transform3d cameraToRobot =
        new Transform3d(new Translation3d(-0.5, 0.0, -0.25), new Rotation3d(0, 0, 0));
    Translation3d robotToObject = detection.getRobotToObject(cameraToRobot);
    assertEquals(2.0, robotToObject.getX(), DELTA);
    assertEquals(-0.5, robotToObject.getY(), DELTA);
    assertEquals(0.0, robotToObject.getZ(), DELTA);

    Transform3d cameraToRobot2 =
        new Transform3d(new Translation3d(-0.5, 0.0, -0.25), new Rotation3d(0, -0.5, 0));
    Translation3d robotToObject2 = detection.getRobotToObject(cameraToRobot2);
    assertEquals(1.75, robotToObject2.getX(), DELTA);
    assertEquals(-0.5, robotToObject2.getY(), DELTA);
    assertEquals(-0.96, robotToObject2.getZ(), DELTA);

    Translation3d fieldToObject =
        detection.getFieldToObject(cameraToRobot, new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90)));
    assertEquals(3.5, fieldToObject.getX(), DELTA);
    assertEquals(5.0, fieldToObject.getY(), DELTA);
    assertEquals(0, fieldToObject.getZ(), DELTA);

    Translation3d fieldToObject2 =
        detection.getFieldToObject(
            cameraToRobot2, new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90)));
    assertEquals(3.5, fieldToObject2.getX(), DELTA);
    assertEquals(4.75, fieldToObject2.getY(), DELTA);
    assertEquals(-0.96, fieldToObject2.getZ(), DELTA);
  }

  @Test
  public void testVelocityTransforms() {
    ObjectDetection detection =
        new ObjectDetection(
            "test",
            0,
            0.8,
            ObjectDetection.TrackingState.OFF,
            true,
            new Translation3d(1.5, -0.5, -0.25),
            new Vector3(2, 1, 0),
            new Vector3(0, 0, 0),
            5.0);

    Vector3 cameraRelativeVelocity = detection.getCameraRelativeVelocity();
    assertEquals(2, cameraRelativeVelocity.getX(), DELTA);
    assertEquals(1, cameraRelativeVelocity.getY(), DELTA);
    assertEquals(0, cameraRelativeVelocity.getZ(), DELTA);

    Transform3d cameraToRobot =
        new Transform3d(new Translation3d(-0.5, 0.0, -0.25), new Rotation3d(0, 0, 0));
    Vector3 robotRelativeVelocity = detection.getRobotRelativeVelocity(cameraToRobot);
    assertEquals(2, robotRelativeVelocity.getX(), DELTA);
    assertEquals(1, robotRelativeVelocity.getY(), DELTA);
    assertEquals(0, robotRelativeVelocity.getZ(), DELTA);

    Transform3d cameraToRobot2 =
        new Transform3d(new Translation3d(-0.5, 0.0, -0.25), new Rotation3d(0, -0.5, 0));
    Vector3 robotRelativeVelocity2 = detection.getRobotRelativeVelocity(cameraToRobot2);
    assertEquals(1.75, robotRelativeVelocity2.getX(), DELTA);
    assertEquals(1, robotRelativeVelocity2.getY(), DELTA);
    assertEquals(-0.96, robotRelativeVelocity2.getZ(), DELTA);

    Vector3 fieldRelativeVelocity =
        detection.getFieldRelativeVelocity(
            cameraToRobot, new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90)));
    assertEquals(-1, fieldRelativeVelocity.getX(), DELTA);
    assertEquals(2, fieldRelativeVelocity.getY(), DELTA);
    assertEquals(0, fieldRelativeVelocity.getZ(), DELTA);

    Vector3 fieldRelativeVelocity2 =
        detection.getFieldRelativeVelocity(
            cameraToRobot2, new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(90)));
    assertEquals(-1, fieldRelativeVelocity2.getX(), DELTA);
    assertEquals(1.75, fieldRelativeVelocity2.getY(), DELTA);
    assertEquals(-0.96, fieldRelativeVelocity2.getZ(), DELTA);
  }
}
