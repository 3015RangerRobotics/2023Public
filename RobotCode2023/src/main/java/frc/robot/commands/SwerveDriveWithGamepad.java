package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDStrip;
import frc.robot.util.ScoringTracker;
import java.util.Optional;

public class SwerveDriveWithGamepad extends CommandBase {
  private final SlewRateLimiter xVelLimiter;
  private final SlewRateLimiter yVelLimiter;
  private final SlewRateLimiter angularVelLimiter;

  private Rotation2d rotationTarget = null;
  private final double rotationHoldFactor;

  private final boolean aimAtGamePiece;

  public SwerveDriveWithGamepad(boolean aimAtGamePiece) {
    this.xVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.yVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.angularVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAngularAccelTele);
    this.rotationHoldFactor = Constants.Swerve.teleAngleHoldFactor;
    this.aimAtGamePiece = aimAtGamePiece;

    addRequirements(RobotContainer.swerve);
  }

  public SwerveDriveWithGamepad() {
    this(false);
  }

  @Override
  public void initialize() {
    ChassisSpeeds currentSpeeds = RobotContainer.swerve.getCurrentSpeeds();
    Translation2d hack =
        new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
            .rotateBy(RobotContainer.swerve.getPose().getRotation());
    this.xVelLimiter.reset(hack.getX());
    this.yVelLimiter.reset(hack.getY());
    this.angularVelLimiter.reset(currentSpeeds.omegaRadiansPerSecond);

    rotationTarget = null;
  }

  @Override
  public void execute() {
    double x = -RobotContainer.driver.getLeftY();
    x = Math.copySign(x * x, x);
    double y = -RobotContainer.driver.getLeftX();
    y = Math.copySign(y * y, y);
    double rot = -RobotContainer.driver.getRightX();
    rot = Math.copySign(rot * rot, rot);

    double targetAngularVel = rot * Constants.Swerve.maxAngularVelTele;
    boolean targeting = false;
    boolean stop = x == 0 && y == 0 && rot == 0;

    // Only take over game piece aim if driver is not rotating and intake is spinning (we don't have
    // a game piece in intake)
    if (aimAtGamePiece
        && RobotContainer.isPickupLLEnabled()
        && rot == 0
        && Math.abs(RobotContainer.intake.getIntakeMotorRPM()) > 100) {
      var detections =
          LimelightHelpers.getLatestResults("limelight-center").targetingResults.targets_Detector;

      boolean cubeMode = RobotContainer.isCubeMode();
      LimelightHelpers.LimelightTarget_Detector target = null;

      for (var det : detections) {
        if (cubeMode && det.className.equals("cube")) {
          if (target == null || det.ta > target.ta) {
            target = det;
          }
        } else if (!cubeMode && det.className.equals("cone")) {
          if (target == null || det.ta > target.ta) {
            target = det;
          }
        }
      }

      if (target != null) {
        targetAngularVel = -Units.degreesToRadians(target.tx) * rotationHoldFactor;
        targeting = true;
      }
    }

    double xVel = this.xVelLimiter.calculate(x * Constants.Swerve.maxVelTele);
    double yVel = this.yVelLimiter.calculate(y * Constants.Swerve.maxVelTele);
    double angularVel = this.angularVelLimiter.calculate(targetAngularVel);

    if (stop) {
      rotationTarget = null;

      RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
    } else {
      if (!aimAtGamePiece && Math.abs(angularVel) <= 0.01) {
        if (rotationTarget == null) {
          rotationTarget = RobotContainer.swerve.getPose().getRotation();
        }

        if (Math.abs(xVel) > 0.2 || Math.abs(yVel) > 0.2) {
          Rotation2d error = rotationTarget.minus(RobotContainer.swerve.getPose().getRotation());
          angularVel = error.getRadians() * rotationHoldFactor;
        }
      } else {
        rotationTarget = null;
      }

      if (!targeting) {
        RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
      } else {
        if (Math.abs(
                new Translation2d(xVel, yVel)
                    .getAngle()
                    .minus(RobotContainer.swerve.getPose().getRotation())
                    .getDegrees())
            < 45.0) {
          double vel = Math.sqrt((xVel * xVel) + (yVel * yVel));
          RobotContainer.swerve.driveRobotRelative(new ChassisSpeeds(vel, 0.0, angularVel));
        } else {
          RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
        }
      }
    }

    Optional<Integer> override = ScoringTracker.getScoringTrackerOverride();
    if (override.isPresent()) {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red
          && (override.get() == 0 || override.get() == 9)) {
        RobotContainer.leds.setState(LEDStrip.State.NORMAL);
        return;
      } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue
          && (override.get() == 8 || override.get() == 17)) {
        RobotContainer.leds.setState(LEDStrip.State.NORMAL);
        return;
      }
    }
    ScoringTracker.getScoringTrackerOverridePosition()
        .ifPresentOrElse(
            (Translation3d scorePos) -> {
              Pose2d robotPose = RobotContainer.swerve.getPose();

              Rotation2d goalTurret =
                  scorePos
                      .toTranslation2d()
                      .minus(robotPose.getTranslation())
                      .getAngle()
                      .minus(robotPose.getRotation());

              double aimPointForward =
                  RobotContainer.isCubeMode()
                      ? Constants.PointToPosition.cubeForward
                      : Constants.PointToPosition.coneForward;

              Translation3d jointPos =
                  new Translation3d(
                      RobotContainer.swerve.getPose().getX(),
                      RobotContainer.swerve.getPose().getY(),
                      Constants.PointToPosition.jointHeight);
              double distanceToTarget = jointPos.getDistance(scorePos);

              double goalExtension =
                  distanceToTarget
                      - Constants.PointToPosition.jointToExtensionLength
                      - aimPointForward;

              if (Math.abs(goalTurret.getDegrees()) < (RobotContainer.isCubeMode() ? 15 : 20)
                  && goalExtension < Constants.PinkArm.Extension.FORWARD_LIMIT - 0.05) {
                RobotContainer.leds.setState(LEDStrip.State.READY_TO_AIM);

                if (RobotContainer.driver.LT().getAsBoolean() && stop) {
                  RobotContainer.swerve.lockModules();
                }
              } else {
                RobotContainer.leds.setState(LEDStrip.State.NORMAL);
              }
            },
            () -> RobotContainer.leds.setState(LEDStrip.State.NORMAL));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
