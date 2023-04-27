package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmToLoadingStationPos extends CommandBase {

  public ArmToLoadingStationPos() {
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.pinkArm.setArmGoalPosition(
        Constants.PinkArm.Extension.extensionLengthChute, Constants.PinkArm.Joint.jointAngleChute);
    RobotContainer.turret.setDesiredAngle(Constants.Turret.turretAngleChute);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
