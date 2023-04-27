package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Util {
  public static Translation3d trans3dFromInches(double inchesX, double inchesY, double inchesZ) {
    return new Translation3d(
        Units.inchesToMeters(inchesX),
        Units.inchesToMeters(inchesY),
        Units.inchesToMeters(inchesZ));
  }

  public static Pose3d pose2dTo3d(Pose2d pose) {
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0.0),
        new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
  }
}
