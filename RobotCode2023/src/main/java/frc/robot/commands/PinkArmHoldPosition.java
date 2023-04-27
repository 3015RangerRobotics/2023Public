package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PinkArmHoldPosition extends CommandBase {
  protected double extensionPos;
  protected double jointPos;

  public PinkArmHoldPosition() {
    addRequirements(RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {
    extensionPos = RobotContainer.pinkArm.getExtensionLength();
    jointPos = RobotContainer.pinkArm.getJointAngle();
  }

  @Override
  public void execute() {
    RobotContainer.pinkArm.setArmGoalPosition(extensionPos, jointPos);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
