package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Jaw;
import java.util.List;

public class TwoPlusBalanceLoading extends SequentialCommandGroup {
  public TwoPlusBalanceLoading(SwerveAutoBuilder autoBuilder, boolean isBlue) {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup(
            "2+Balance Loading" + (isBlue ? "Blue" : ""),
            new PathConstraints(3, 2.5),
            new PathConstraints(3, 3),
            new PathConstraints(3, 2.5),
            new PathConstraints(3, 3));

    addCommands(
        Commands.sequence(
                AutoBuildingBlocks.scoreStartConeBehindLoading(),
                autoBuilder.resetPose(pathGroup.get(0)),
                RobotContainer.pinkArm.setExtensionLengthCommand(0.0),
                AutoBuildingBlocks.setIsCubeMode(true),
                autoBuilder.followPathWithEvents(
                    pathGroup.get(0)), // Events: intakeFront, aimAtGamePiece
                AutoBuildingBlocks.stopAimAtGamePiece(),
                RobotContainer.jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                autoBuilder
                    .followPathWithEvents(pathGroup.get(1))
                    .raceWith(
                        RobotContainer.intake.intakeHoldCommand()), // Events: preAimHighCubeBehind,
                // preAimHighCubeBehind2
                Commands.waitSeconds(0.4),
                AutoBuildingBlocks.outtakeGamePiece(),
                RobotContainer.pinkArm.setExtensionLengthCommand(0.0),
                autoBuilder.followPathWithEvents(
                    pathGroup.get(2)), // Events: intakeFront, aimAtGamePiece
                AutoBuildingBlocks.stopAimAtGamePiece(),
                RobotContainer.pinkArm.setExtensionLengthCommand(0),
                autoBuilder
                    .followPathWithEvents(pathGroup.get(3))
                    .raceWith(RobotContainer.intake.intakeHoldCommand()), // Events: armToZero
                RobotContainer.swerve.autoBalance(Constants.AutoBalance.maxVelAuto))
            .withTimeout(14.9)
            .andThen(Commands.run(RobotContainer.swerve::lockModules, RobotContainer.swerve)));
  }
}
