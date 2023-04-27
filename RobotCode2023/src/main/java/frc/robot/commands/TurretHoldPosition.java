package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurretHoldPosition extends CommandBase {
  protected double turretPos;

  public TurretHoldPosition() {
    addRequirements(RobotContainer.turret);
  }

  @Override
  public void initialize() {
    turretPos = RobotContainer.turret.getTurretAngle();
  }

  @Override
  public void execute() {
    RobotContainer.turret.setDesiredAngle(turretPos);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
