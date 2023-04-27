package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Jaw;

public class ScoreLowManual extends CommandBase {

  public ScoreLowManual() {
    addRequirements(RobotContainer.turret, RobotContainer.pinkArm, RobotContainer.jaw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (RobotContainer.isCubeMode()) {
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.ScoreLowManual.cubePinkArmLength, Constants.ScoreLowManual.cubePinkArmAngle);
    } else {
      RobotContainer.pinkArm.setArmGoalPosition(
          Constants.ScoreLowManual.conePinkArmLength, Constants.ScoreLowManual.conePinkArmAngle);
    }
    RobotContainer.jaw.setJawStateCommand(Jaw.State.SCORING_POS);
    RobotContainer.turret.setDesiredAngle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
