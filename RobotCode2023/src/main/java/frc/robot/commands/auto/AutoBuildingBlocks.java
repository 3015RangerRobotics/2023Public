package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.PointToPositionAutoOnly;
import frc.robot.subsystems.Jaw;
import frc.robot.util.ScoringTracker;

public class AutoBuildingBlocks {
  public static CommandBase scoreStartConeBehindWire() {
    return Commands.sequence(
        RobotContainer.jaw.setJawStateCommand(Jaw.State.IDLE),
        Commands.runOnce(() -> RobotContainer.jaw.setDesiredPivotAngle(300)),
        AutoBuildingBlocks.setIsCubeMode(false),
        Commands.parallel(
                RobotContainer.pinkArm.setArmPositionCommand(0.15, 160),
                allianceConditionalCommand(
                    RobotContainer.turret.setTurretAngleCommand(3.5),
                    RobotContainer.turret.setTurretAngleCommand(-3.5)),
                RobotContainer.intake.intakeHoldCommand())
            .withTimeout(1.0),
        outtakeGamePiece(),
        Commands.runOnce(
            () -> {
              RobotContainer.pinkArm.setArmGoalPosition(0.0, 0.0);
              RobotContainer.turret.setDesiredAngle(0.0);
            },
            RobotContainer.pinkArm,
            RobotContainer.turret));
  }

  public static CommandBase scoreStartConeBehindLoading() {
    return Commands.sequence(
        RobotContainer.jaw.setJawStateCommand(Jaw.State.IDLE),
        Commands.runOnce(() -> RobotContainer.jaw.setDesiredPivotAngle(300)),
        AutoBuildingBlocks.setIsCubeMode(false),
        Commands.parallel(
                RobotContainer.pinkArm.setArmPositionCommand(0.17, 160),
                allianceConditionalCommand(
                    RobotContainer.turret.setTurretAngleCommand(-1.0),
                    RobotContainer.turret.setTurretAngleCommand(2.5)),
                RobotContainer.intake.intakeHoldCommand())
            .withTimeout(1.0),
        outtakeGamePiece(),
        Commands.runOnce(
            () -> {
              RobotContainer.pinkArm.setArmGoalPosition(0.0, 0.0);
              RobotContainer.turret.setDesiredAngle(0.0);
            },
            RobotContainer.pinkArm,
            RobotContainer.turret));
  }

  static CommandBase aimTurretToHighCubeLL() {
    return Commands.run(
        () -> {
          DriverStation.Alliance alliance = DriverStation.getAlliance();

          Pose2d robotPose;
          Translation3d targetPose;

          var fid = LimelightHelpers.getFiducialID("limelight-rear");

          if (alliance == DriverStation.Alliance.Red) {
            robotPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 1 ? 1 : 7, alliance);
          } else {
            robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 6 ? 1 : 7, alliance);
          }

          if (robotPose.equals(new Pose2d())) {
            return;
          }

          Rotation2d turretAngleFieldRelative =
              targetPose
                  .toTranslation2d()
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Rotation2d.fromDegrees(180));

          if (Math.abs(turretAngleFieldRelative.minus(Rotation2d.fromDegrees(180)).getDegrees())
              > 10) {
            RobotContainer.turret.setDesiredAngleFieldRelative(
                turretAngleFieldRelative, RobotContainer.swerve.getPose());
          }
        },
        RobotContainer.turret);
  }

  static CommandBase aimTurretToMidCubeLL() {
    return Commands.run(
        () -> {
          DriverStation.Alliance alliance = DriverStation.getAlliance();

          Pose2d robotPose;
          Translation3d targetPose;

          var fid = LimelightHelpers.getFiducialID("limelight-rear");

          if (alliance == DriverStation.Alliance.Red) {
            robotPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 1 ? 10 : 16, alliance);
          } else {
            robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 6 ? 10 : 16, alliance);
          }

          if (robotPose.equals(new Pose2d())) {
            return;
          }

          Rotation2d turretAngleFieldRelative =
              targetPose
                  .toTranslation2d()
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Rotation2d.fromDegrees(180));

          if (Math.abs(turretAngleFieldRelative.minus(Rotation2d.fromDegrees(180)).getDegrees())
              > 10) {
            RobotContainer.turret.setDesiredAngleFieldRelative(
                turretAngleFieldRelative, RobotContainer.swerve.getPose());
          }
        },
        RobotContainer.turret);
  }

  static CommandBase pointToPosition(int rowIdx, int redColIdx, int blueColIdx) {
    return Commands.parallel(
        allianceConditionalCommand(
            new PointToPositionAutoOnly(
                ScoringTracker.getScoringPosAlliance(rowIdx, redColIdx, DriverStation.Alliance.Red),
                true),
            new PointToPositionAutoOnly(
                ScoringTracker.getScoringPosAlliance(
                    rowIdx, blueColIdx, DriverStation.Alliance.Blue),
                true)),
        RobotContainer.jaw.setJawStateCommand(Jaw.State.SCORING_POS),
        RobotContainer.intake.intakeCommand());
  }

  static CommandBase outtakeGamePiece() {
    return outtakeGamePiece(0.25);
  }

  static CommandBase outtakeGamePiece(double outtakeTime) {
    return RobotContainer.intake
        .outtakeCommand()
        .withTimeout(outtakeTime)
        .andThen(RobotContainer.intake.stopIntakeCommand());
  }

  static CommandBase setIsCubeMode(boolean isCubeMode) {
    return Commands.runOnce(() -> RobotContainer.setIsCubeMode(isCubeMode));
  }

  static CommandBase stopAimAtGamePiece() {
    return Commands.runOnce(
        () -> {
          Autos.aimAtGamePiece = false;
        });
  }

  static CommandBase aimAtGamePiece() {
    return Commands.runOnce(
        () -> {
          Autos.aimAtGamePiece = true;
        });
  }

  static CommandBase allianceConditionalCommand(CommandBase redCommand, CommandBase blueCommand) {
    return new ConditionalCommand(
        redCommand, blueCommand, () -> DriverStation.getAlliance() == DriverStation.Alliance.Red);
  }
}
