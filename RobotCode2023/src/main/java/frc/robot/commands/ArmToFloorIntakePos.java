package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmToFloorIntakePos extends CommandBase {
  private final boolean tipped;

  public ArmToFloorIntakePos(boolean tipped) {
    this.tipped = tipped;
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotContainer.isCubeMode()) {
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.PinkArm.Extension.cubeFloorIntakeLength,
          Constants.PinkArm.Joint.cubeFloorIntakeAngle);
      RobotContainer.turret.setDesiredAngle((0));
    } else {
      RobotContainer.pinkArm.setArmGoalPosition(
          tipped
              ? Constants.PinkArm.Extension.coneTippedFloorIntakeLength
              : Constants.PinkArm.Extension.coneFloorIntakeLength,
          tipped
              ? Constants.PinkArm.Joint.coneTippedFloorIntakeAngle
              : Constants.PinkArm.Joint.coneFloorIntakeAngle);
      RobotContainer.turret.setDesiredAngle((0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
