package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmToShelfPos extends CommandBase {

  public ArmToShelfPos() {
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.pinkArm.setArmGoalPosition(
        Constants.PinkArm.Extension.extensionLengthShelf, Constants.PinkArm.Joint.jointAngleShelf);
    RobotContainer.turret.setDesiredAngle(Constants.Turret.turretAngleShelf);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
