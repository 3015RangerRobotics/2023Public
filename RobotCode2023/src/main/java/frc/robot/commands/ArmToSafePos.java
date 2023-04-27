package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmToSafePos extends CommandBase {
  private final boolean forceUp;

  protected enum State {
    EXTENSION_TO_SAFE,
    ARM_TO_SAFE,
    ALL_TO_SAFE
  }

  protected State state;

  protected double startTurret;
  protected double startJoint;

  public ArmToSafePos(boolean forceUp) {
    this.forceUp = forceUp;

    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  public ArmToSafePos() {
    this(false);
  }

  @Override
  public void initialize() {
    state = State.EXTENSION_TO_SAFE;

    startJoint = RobotContainer.pinkArm.getJointAngle();
    startTurret = RobotContainer.turret.getTurretAngle();
  }

  @Override
  public void execute() {
    boolean nearCommunity =
        RobotContainer.swerve.getPose().getX() < Constants.SafePositions.safeFieldPosition;
    double safeJoint =
        (nearCommunity && !forceUp
                || (DriverStation.getMatchTime() != -1 && DriverStation.getMatchTime() < 15))
            ? Constants.SafePositions.safeJointAngleCommunity
            : Constants.SafePositions.safeJointAngle;

    switch (state) {
      case EXTENSION_TO_SAFE:
        RobotContainer.pinkArm.setArmGoalPosition(
            Constants.SafePositions.safeArmExtension,
            startJoint + 10); // Move arm up a bit to prevent accidental unscoring
        RobotContainer.turret.setDesiredAngle(startTurret);

        if (RobotContainer.pinkArm.getExtensionLength() <= 0.2) {
          state = State.ARM_TO_SAFE;
        }
        break;
      case ARM_TO_SAFE:
        RobotContainer.pinkArm.setArmGoalPosition(
            Constants.SafePositions.safeArmExtension, safeJoint);
        RobotContainer.turret.setDesiredAngle(startTurret);

        if (Math.abs(RobotContainer.pinkArm.getJointAngle() - safeJoint) <= 10.0) {
          state = State.ALL_TO_SAFE;
        }
        break;
      case ALL_TO_SAFE:
        RobotContainer.pinkArm.setArmGoalPosition(
            Constants.SafePositions.safeArmExtension, safeJoint);
        RobotContainer.turret.setDesiredAngle(Constants.SafePositions.safeTurretAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
