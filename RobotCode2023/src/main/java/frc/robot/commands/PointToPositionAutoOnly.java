package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.IntakeCam;
import frc.robot.RobotContainer;

public class PointToPositionAutoOnly extends CommandBase {
  private Translation3d position;
  private final boolean correct;

  private double intakeOffset;

  public PointToPositionAutoOnly(Translation3d position, boolean correct) {
    this.position = position;
    this.correct = correct;

    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  public PointToPositionAutoOnly(Translation3d position) {
    this(position, true);
  }

  @Override
  public void initialize() {
    intakeOffset = RobotContainer.isCubeMode() ? 0 : IntakeCam.getConeOffsetMeters();

    position =
        new Translation3d(
            position.getX()
                + (RobotContainer.isCubeMode()
                    ? Constants.PointToPosition.scoringTrackerCubeXOffset
                    : Constants.PointToPosition.scoringTrackerConeXOffset),
            position.getY(),
            position.getZ()
                + (RobotContainer.isCubeMode()
                    ? Constants.PointToPosition.scoringTrackerCubeZOffset
                    : Constants.PointToPosition.scoringTrackerConeZOffset));
  }

  @Override
  public void execute() {
    if (correct) {
      RobotContainer.swerve.correctOdom(false, false);
    }

    RobotContainer.turret.pointTurretToPosition(position, intakeOffset);
    RobotContainer.pinkArm.pointArmJointToPosition(position);
    RobotContainer.pinkArm.pointArmExtensionToPosition(position);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
