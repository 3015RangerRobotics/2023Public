package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;

public class ManhattanClimb extends ParallelCommandGroup {
  public ManhattanClimb() {
    addCommands(
        RobotContainer.turret.setTurretAngleCommand(0),
        RobotContainer.pinkArm.setArmPositionCommand(0, 180),
        Commands.sequence(
            RobotContainer.manhattan
                .setArmVoltage(-6)
                .until(() -> RobotContainer.manhattan.getArmAngle() <= -40),
            RobotContainer.manhattan.stopArm(),
            Commands.parallel(
                    Commands.run(
                        () ->
                            RobotContainer.swerve.driveRobotRelative(
                                new ChassisSpeeds(-0.2, 0, 0))),
                    RobotContainer.manhattan.setArmVoltage(8))
                .until(() -> RobotContainer.manhattan.getArmAngle() >= 35),
            RobotContainer.manhattan.stopArm(),
            Commands.run(
                    () -> RobotContainer.swerve.driveRobotRelative(new ChassisSpeeds(-0.85, 0, 0)))
                .alongWith(RobotContainer.manhattan.setTankSpeed(1.0))
                .withTimeout(2.0),
            Commands.run(
                    () -> RobotContainer.swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0, 0)))
                .alongWith(RobotContainer.manhattan.setTankSpeed(0.0))));
  }
}
