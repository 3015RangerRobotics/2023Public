package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Vector3;
import frc.robot.RobotContainer;

public class UntipRobot extends CommandBase {
  private enum TipDirection {
    ON_FRONT,
    ON_LEFT,
    ON_RIGHT,
    ON_BACK
  }

  private TipDirection tipDirection;

  public UntipRobot() {
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {
    Vector3 gravVector = RobotContainer.swerve.getGravityVector();

    if (Math.abs(gravVector.getX()) >= Math.abs(gravVector.getY())) {
      // Either on front or back
      if (gravVector.getX() < 0) {
        tipDirection = TipDirection.ON_FRONT;
      } else {
        tipDirection = TipDirection.ON_BACK;
      }
    } else {
      // On left or right
      if (gravVector.getY() > 0) {
        tipDirection = TipDirection.ON_LEFT;
      } else {
        tipDirection = TipDirection.ON_RIGHT;
      }
    }
  }

  @Override
  public void execute() {
    if (tipDirection != null) {
      switch (tipDirection) {
        case ON_BACK:
          RobotContainer.turret.setDesiredAngle(0);
          if (Math.abs(RobotContainer.turret.getTurretAngle()) < 15) {
            RobotContainer.pinkArm.setArmGoalPosition(0.0, 190);
          }
          break;
        case ON_FRONT:
          RobotContainer.turret.setDesiredAngle(0);
          if (Math.abs(RobotContainer.turret.getTurretAngle()) < 15) {
            RobotContainer.pinkArm.setArmGoalPosition(0.0, -20);
          }
          break;
        case ON_LEFT:
          RobotContainer.turret.setDesiredAngle(-60);
          if (Math.abs(RobotContainer.turret.getTurretAngle() + 60) < 15) {
            RobotContainer.pinkArm.setArmGoalPosition(0.0, -20);
          }
          break;
        case ON_RIGHT:
          RobotContainer.turret.setDesiredAngle(60);
          if (Math.abs(RobotContainer.turret.getTurretAngle() - 60) < 15) {
            RobotContainer.pinkArm.setArmGoalPosition(0.0, -20);
          }
          break;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
