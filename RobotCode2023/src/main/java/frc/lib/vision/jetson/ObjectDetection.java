package frc.lib.vision.jetson;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.util.Vector3;

public class ObjectDetection {
  private final String className;
  private final int objectID;
  private final double confidence;
  private final TrackingState trackingState;
  private final boolean isMoving;
  private final Translation3d cameraToObject;
  private final Vector3 cameraRelativeVelocity;
  private final Vector3 dimensions;
  private final double timestampSeconds;

  public enum TrackingState {
    OFF,
    OK,
    SEARCHING,
    TERMINATE
  }

  public ObjectDetection(
      String className,
      int objectID,
      double confidence,
      TrackingState trackingState,
      boolean isMoving,
      Translation3d cameraRelativeTranslation,
      Vector3 cameraRelativeVelocity,
      Vector3 dimensions,
      double timestampSeconds) {
    this.className = className;
    this.objectID = objectID;
    this.confidence = confidence;
    this.trackingState = trackingState;
    this.isMoving = isMoving;
    this.cameraToObject = cameraRelativeTranslation;
    this.cameraRelativeVelocity = cameraRelativeVelocity;
    this.dimensions = dimensions;
    this.timestampSeconds = timestampSeconds;
  }

  public String getClassName() {
    return this.className;
  }

  public int getObjectID() {
    return this.objectID;
  }

  public double getConfidence() {
    return this.confidence;
  }

  public TrackingState getTrackingState() {
    return this.trackingState;
  }

  public boolean isMoving() {
    return this.isMoving;
  }

  public Translation3d getCameraToObject() {
    return this.cameraToObject;
  }

  public Translation3d getRobotToObject(Transform3d cameraToRobot) {
    Transform3d robotToCamera = cameraToRobot.inverse();
    return robotToCamera
        .plus(new Transform3d(getCameraToObject(), new Rotation3d()))
        .getTranslation();
  }

  public Translation3d getFieldToObject(Transform3d cameraToRobot, Pose2d robotPose) {
    Translation3d robotToObj = getRobotToObject(cameraToRobot);
    Transform3d fieldToRobot =
        new Transform3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), 0),
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d fieldToObj = fieldToRobot.plus(new Transform3d(robotToObj, new Rotation3d()));
    return fieldToObj.getTranslation();
  }

  public Vector3 getCameraRelativeVelocity() {
    if (!this.isMoving()) {
      return new Vector3(0, 0, 0);
    }
    return this.cameraRelativeVelocity;
  }

  public Vector3 getRobotRelativeVelocity(Transform3d cameraToRobot) {
    // Rotation only to transform a vector since it is not a coordinate
    Vector3 cameraRelVel = getCameraRelativeVelocity();
    Translation3d cameraVel =
        new Translation3d(cameraRelVel.getX(), cameraRelVel.getY(), cameraRelVel.getZ());
    Translation3d robotVel = cameraVel.rotateBy(cameraToRobot.inverse().getRotation());
    return new Vector3(robotVel.getX(), robotVel.getY(), robotVel.getZ());
  }

  public Vector3 getFieldRelativeVelocity(Transform3d cameraToRobot, Pose2d robotPose) {
    // Rotation only to transform a vector since it is not a coordinate
    Vector3 robotRelVel = getRobotRelativeVelocity(cameraToRobot);
    Translation3d robotVel =
        new Translation3d(robotRelVel.getX(), robotRelVel.getY(), robotRelVel.getZ());
    Translation3d fieldVel =
        robotVel.rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
    return new Vector3(fieldVel.getX(), fieldVel.getY(), fieldVel.getZ());
  }

  public Vector3 getDimensions() {
    return this.dimensions;
  }

  public double getTimestampSeconds() {
    return this.timestampSeconds;
  }
}
