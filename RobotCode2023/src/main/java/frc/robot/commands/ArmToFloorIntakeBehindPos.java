package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmToFloorIntakeBehindPos extends CommandBase {
  protected enum State {
    EXTENSION_RETRACT,
    JOINT_ROTATE,
    EXTENSION_EXTEND
  }

  protected State state;
  protected double jointStart;

  public ArmToFloorIntakeBehindPos() {
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {
    state = State.EXTENSION_RETRACT;
    jointStart = RobotContainer.pinkArm.getJointAngle();
  }

  @Override
  public void execute() {
    RobotContainer.turret.setDesiredAngle((0));

    double goalExtension =
        RobotContainer.isCubeMode()
            ? Constants.PinkArm.Extension.cubeFloorIntakeBehindLength
            : Constants.PinkArm.Extension.coneFloorIntakeBehindLength;
    double goalJoint =
        RobotContainer.isCubeMode()
            ? Constants.PinkArm.Joint.cubeFloorIntakeBehindAngle
            : Constants.PinkArm.Joint.coneFloorIntakeBehindAngle;

    switch (state) {
      case EXTENSION_RETRACT:
        RobotContainer.pinkArm.setArmGoalPosition(0.0, jointStart);
        if (Math.abs(RobotContainer.pinkArm.getExtensionLength()) < 0.25) {
          state = State.JOINT_ROTATE;
        }
        break;
      case JOINT_ROTATE:
        RobotContainer.pinkArm.setArmGoalPosition(0.0, goalJoint);
        if (Math.abs(RobotContainer.pinkArm.getJointAngle() - goalJoint) < 45) {
          state = State.EXTENSION_EXTEND;
        }
        break;
      case EXTENSION_EXTEND:
        RobotContainer.pinkArm.setArmGoalPosition(goalExtension, goalJoint);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
