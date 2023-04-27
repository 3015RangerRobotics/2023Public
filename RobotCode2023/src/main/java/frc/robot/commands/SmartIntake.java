package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleOn;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Jaw;

public class SmartIntake extends CommandBase {
  private final boolean tipped;

  public SmartIntake(boolean tipped) {
    this.tipped = tipped;

    addRequirements(
        RobotContainer.intake, RobotContainer.jaw, RobotContainer.turret, RobotContainer.pinkArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Constants.IntakeMode intakeMode = RobotContainer.getIntakeMode();
    boolean cubeMode = RobotContainer.isCubeMode();

    switch (intakeMode) {
      case SHELF:
        RobotContainer.pinkArm.setArmGoalPosition(
            Constants.PinkArm.Extension.extensionLengthShelf,
            Constants.PinkArm.Joint.jointAngleShelf);
        RobotContainer.turret.setDesiredAngle(0.0);
        RobotContainer.jaw.setState(Jaw.State.SHELF_PICKUP);
        break;
      case CHUTE:
        RobotContainer.pinkArm.setArmGoalPosition(
            Constants.PinkArm.Extension.extensionLengthChute,
            Constants.PinkArm.Joint.jointAngleChute);
        RobotContainer.turret.setDesiredAngle(0.0);
        RobotContainer.jaw.setState(Jaw.State.CHUTE_PICKUP);
        break;
      case FLOOR:
        RobotContainer.pinkArm.setArmGoalPosition(
            cubeMode
                ? Constants.PinkArm.Extension.cubeFloorIntakeLength
                : Constants.PinkArm.Extension.coneFloorIntakeLength,
            cubeMode
                ? Constants.PinkArm.Joint.cubeFloorIntakeAngle
                : tipped
                    ? Constants.PinkArm.Joint.coneTippedFloorIntakeAngle
                    : Constants.PinkArm.Joint.coneFloorIntakeAngle);
        RobotContainer.turret.setDesiredAngle(0.0);
        RobotContainer.jaw.setState(
            tipped ? Jaw.State.FLOOR_TIPPED_PICKUP : Jaw.State.FLOOR_PICKUP);
    }

    if (DriverStation.isTeleopEnabled()) {
      if (Math.abs(RobotContainer.intake.getIntakeMotorRPM()) < 60) {
        RobotContainer.driver.setRumbleAnimation(new RumbleOn());
      } else {
        RobotContainer.driver.setRumbleAnimation(new RumbleOff());
      }
    }

    RobotContainer.intake.setSpeed(
        cubeMode ? Constants.Intake.intakeSpeedCube : Constants.Intake.intakeSpeedCone);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    boolean cubeMode = RobotContainer.isCubeMode();
    RobotContainer.jaw.setState(Jaw.State.SCORING_POS);
    RobotContainer.intake.setSpeed(
        cubeMode ? Constants.Intake.intakeHoldPercentCube : Constants.Intake.intakeHoldPercentCone);
  }
}
