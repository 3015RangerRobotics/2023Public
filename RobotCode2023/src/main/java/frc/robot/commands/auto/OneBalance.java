package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;

public class OneBalance extends SequentialCommandGroup {
  public OneBalance(SwerveAutoBuilder autoBuilder) {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("1MidBalance", new PathConstraints(1, 2));

    addCommands(
        Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                AutoBuildingBlocks.setIsCubeMode(true),
                RobotContainer.turret
                    .setTurretAngleCommand(0)
                    .raceWith(
                        Commands.sequence(
                            RobotContainer.pinkArm.setArmPositionCommand(0.0, 155).withTimeout(1.0),
                            RobotContainer.pinkArm
                                .setArmPositionCommand(0.4, 155)
                                .withTimeout(1.0))),
                AutoBuildingBlocks.outtakeGamePiece(),
                RobotContainer.pinkArm.setExtensionLengthCommand(0),
                Commands.waitSeconds(0.5),
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                RobotContainer.swerve.autoBalance(Constants.AutoBalance.maxVelAuto))
            .withTimeout(14.9)
            .andThen(Commands.run(RobotContainer.swerve::lockModules, RobotContainer.swerve)));
  }
}
