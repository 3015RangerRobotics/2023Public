package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.IntakeCam;
import frc.robot.RobotContainer;
import frc.robot.util.ScoringTracker;
import java.util.Optional;

public class ArmToScoringTrackerPos extends CommandBase {
  private double intakeOffset;
  private final boolean overrideOnly;

  private double overrideTurretAngle;
  private double overrideArmAngle;
  private double overrideExtensionLength;

  private boolean llTakeOver;

  public ArmToScoringTrackerPos(boolean overrideOnly) {
    this.overrideOnly = overrideOnly;

    addRequirements(RobotContainer.turret, RobotContainer.pinkArm, RobotContainer.intake);
  }

  @Override
  public void initialize() {
    if (RobotContainer.isPoleLLEnabled() && !RobotContainer.isCubeMode()) {
      LimelightHelpers.setLEDMode_ForceOn("limelight-arm");
    }

    if (!RobotContainer.isCubeMode()) {
      RobotContainer.intake.setSpeed(Constants.Intake.intakeSpeedCone);
    }

    llTakeOver = false;

    intakeOffset = RobotContainer.isCubeMode() ? 0 : IntakeCam.getConeOffsetMeters();

    SmartDashboard.putNumber("LastIntakeOffset", intakeOffset);

    overrideTurretAngle = 0;
    overrideArmAngle = 0;
    overrideExtensionLength = 0;
  }

  @Override
  public void execute() {
    Optional<Translation3d> optScoringPos =
        overrideOnly
            ? ScoringTracker.getScoringTrackerOverridePosition()
            : ScoringTracker.getPreferredScoringPos(RobotContainer.swerve.getPose());
    if (optScoringPos.isEmpty()) {
      RobotContainer.turret.setDesiredAngle(Constants.SafePositions.safeTurretAngle);
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.SafePositions.safeArmExtension,
          Constants.SafePositions.safeJointAngleCommunity);
    } else {
      Translation3d scorePos = optScoringPos.get();
      scorePos =
          new Translation3d(
              scorePos.getX()
                  + (RobotContainer.isCubeMode()
                      ? Constants.PointToPosition.scoringTrackerCubeXOffset
                      : Constants.PointToPosition.scoringTrackerConeXOffset),
              scorePos.getY(),
              scorePos.getZ()
                  + (RobotContainer.isCubeMode()
                      ? Constants.PointToPosition.scoringTrackerCubeZOffset
                      : Constants.PointToPosition.scoringTrackerConeZOffset));

      // Log targeted position to SD
      Quaternion rot = new Rotation3d(0, Units.degreesToRadians(-90), 0).getQuaternion();
      SmartDashboard.putNumberArray(
          "TargetedScoringPos",
          new double[] {
            scorePos.getX(),
            scorePos.getY(),
            scorePos.getZ(),
            rot.getX(),
            rot.getY(),
            rot.getZ(),
            rot.getW()
          });

      if (RobotContainer.driver.A().getAsBoolean()) {
        overrideTurretAngle = RobotContainer.turret.getTurretAngle();
        overrideArmAngle = RobotContainer.pinkArm.getJointAngle();
        overrideExtensionLength = RobotContainer.pinkArm.getExtensionLength();
      }

      Optional<Integer> overrideIndex = ScoringTracker.getScoringTrackerOverride();
      DriverStation.Alliance alliance = DriverStation.getAlliance();
      if (overrideIndex.isPresent()) {
        if (overrideIndex.get() >= 18) {
          if (RobotContainer.isCubeMode()) {
            RobotContainer.pinkArm.setArmGoalPosition(
                Constants.ScoreLowManual.cubePinkArmLength,
                Constants.ScoreLowManual.cubePinkArmAngle);
          } else {
            RobotContainer.pinkArm.setArmGoalPosition(
                Constants.ScoreLowManual.conePinkArmLength,
                Constants.ScoreLowManual.conePinkArmAngle);
          }
          RobotContainer.turret.setDesiredAngle(0);
          return;
        }

        if (alliance == DriverStation.Alliance.Red) {
          if (overrideIndex.get() == 0) {
            aimManualHigh();
            return;
          } else if (overrideIndex.get() == 9) {
            aimManualMid();
            return;
          }
        } else {
          if (overrideIndex.get() == 8) {
            aimManualHigh();
            return;
          } else if (overrideIndex.get() == 17) {
            aimManualMid();
            return;
          }
        }
      }

      if (overrideTurretAngle == 0 && overrideArmAngle == 0 && overrideExtensionLength == 0) {
        if (RobotContainer.isPoleLLEnabled() && !RobotContainer.isCubeMode()) {
          double turretErr =
              Math.abs(
                  RobotContainer.turret.getTurretAngle()
                      - RobotContainer.turret.turretSetpointDegrees);
          double extensionErr =
              Math.abs(
                  RobotContainer.pinkArm.getExtensionLength()
                      - RobotContainer.pinkArm.extensionSetpoint);

          if (turretErr < 10
              && extensionErr < 0.05
              && RobotContainer.pinkArm.extensionSetpoint > 0.05) {
            llTakeOver = true;
          }

          if (llTakeOver) {
            llBasedTurret(true);
          } else {
            RobotContainer.turret.pointTurretToPosition(scorePos, intakeOffset);
          }
        } else {
          RobotContainer.turret.pointTurretToPosition(scorePos, intakeOffset);
        }
        RobotContainer.pinkArm.pointArmJointToPosition(scorePos);
        RobotContainer.pinkArm.pointArmExtensionToPosition(scorePos);
      } else {
        RobotContainer.turret.setDesiredAngle(overrideTurretAngle);
        RobotContainer.pinkArm.setArmPositionCommand(overrideExtensionLength, overrideArmAngle);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumberArray("TargetedScoringPos", new double[] {});
    LimelightHelpers.setLEDMode_ForceOff("limelight-arm");
  }

  private void aimManualHigh() {
    if (RobotContainer.pinkArm.getJointAngle() > 60) {
      RobotContainer.pinkArm.setArmGoalPosition(0.0, Constants.ScoreHighManual.conePinkArmAngle);
    } else {
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.ScoreHighManual.conePinkArmLength, Constants.ScoreHighManual.conePinkArmAngle);
    }

    if (RobotContainer.pinkArm.getExtensionLength()
            < Constants.ScoreHighManual.conePinkArmLength - 0.1
        || !RobotContainer.isPoleLLEnabled()) {
      RobotContainer.turret.setDesiredAngle(0);
    } else {
      llBasedTurret(false);
    }
  }

  private void aimManualMid() {
    if (RobotContainer.pinkArm.getJointAngle() > 60) {
      RobotContainer.pinkArm.setArmGoalPosition(0.0, Constants.ScoreMidManual.conePinkArmAngle);
    } else {
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.ScoreMidManual.conePinkArmLength, Constants.ScoreMidManual.conePinkArmAngle);
    }

    if (RobotContainer.pinkArm.getExtensionLength()
            < Constants.ScoreMidManual.conePinkArmLength - 0.1
        || !RobotContainer.isPoleLLEnabled()) {
      RobotContainer.turret.setDesiredAngle(0);
    } else {
      llBasedTurret(false);
    }
  }

  private void llBasedTurret(boolean autoOuttake) {
    if (!LimelightHelpers.getTV("limelight-arm")) {
      return;
    }
    double llAngleToPole = -LimelightHelpers.getTX("limelight-arm");
    double targetAngle =
        Units.radiansToDegrees(Math.atan(intakeOffset / Constants.CameraInfo.armLLToCone));
    double err = targetAngle - llAngleToPole;
    double o = Math.tan(Units.degreesToRadians(err)) * Constants.CameraInfo.armLLToCone;

    double turretOffset =
        Units.radiansToDegrees(
            Math.atan(
                o
                    / (Constants.CameraInfo.armLLToCone
                        + Constants.CameraInfo.armJointToLL
                        + RobotContainer.pinkArm.getExtensionLength())));

    Optional<Integer> overrideIndex = ScoringTracker.getScoringTrackerOverride();
    double multiplier = 1.0;
    if (overrideIndex.isPresent() && overrideIndex.get() < 9) {
      multiplier = 0.75;
    }

    RobotContainer.turret.setDesiredAngle(
        RobotContainer.turret.getTurretAngle() - (turretOffset * multiplier));

    if (autoOuttake
        && Math.abs(turretOffset) < 1
        && Math.abs(RobotContainer.turret.getTurretVelocity()) < 2) {
      RobotContainer.intake.setSpeed(Constants.Intake.outtakeSpeedCone);
    }
  }
}
