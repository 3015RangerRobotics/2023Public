package frc.robot.commands.auto;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmToFloorIntakeBehindPos;
import frc.robot.commands.ArmToSafePos;
import frc.robot.commands.SmartIntake;
import frc.robot.subsystems.Jaw;
import java.util.HashMap;
import java.util.Map;

public final class Autos {
  private static HashMap<String, Command> eventMap;
  private static SwerveAutoBuilder autoBuilder;
  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  static boolean aimAtGamePiece = false;

  public static void init() {
    eventMap = buildEventMap();

    autoBuilder =
        new SwerveAutoBuilder(
            RobotContainer.swerve::getPose,
            RobotContainer.swerve::resetOdometry,
            Constants.Swerve.PathFollowing.TRANSLATION_CONSTANTS,
            Constants.Swerve.PathFollowing.ROTATION_CONSTANTS,
            (ChassisSpeeds robotRelativeSpeeds) -> {
              if (aimAtGamePiece && Math.abs(RobotContainer.intake.getIntakeMotorRPM()) > 100) {
                var detections =
                    LimelightHelpers.getLatestResults("limelight-center")
                        .targetingResults
                        .targets_Detector;

                boolean cubeMode = RobotContainer.isCubeMode();
                LimelightHelpers.LimelightTarget_Detector target = null;

                for (var det : detections) {
                  if (cubeMode && det.className.equals("cube")) {
                    if ((target == null || det.ta > target.ta) && Math.abs(det.tx) < 20) {
                      target = det;
                    }
                  } else if (!cubeMode && det.className.equals("cone")) {
                    if ((target == null || det.ta > target.ta) && Math.abs(det.tx) < 20) {
                      target = det;
                    }
                  }
                }

                if (target != null) {
                  robotRelativeSpeeds.omegaRadiansPerSecond =
                      -Units.degreesToRadians(target.tx)
                          * Constants.Swerve.PathFollowing.ROTATION_CONSTANTS.kP;
                }

                if (Math.sqrt(
                        (robotRelativeSpeeds.vxMetersPerSecond
                                * robotRelativeSpeeds.vxMetersPerSecond)
                            + (robotRelativeSpeeds.vyMetersPerSecond
                                * robotRelativeSpeeds.vyMetersPerSecond))
                    < 0.5) {
                  robotRelativeSpeeds.omegaRadiansPerSecond = 0.0;
                }
              }

              RobotContainer.swerve.driveRobotRelative(robotRelativeSpeeds);
            },
            eventMap,
            true,
            RobotContainer.swerve);

    autoChooser.setDefaultOption("None", none());
    autoChooser.addOption("2 Balance Loading", twoPlusBalanceLoading());
    autoChooser.addOption("2 Balance Wire", twoPlusBalanceWire());
    autoChooser.addOption("3 Wire", threeWire());
    autoChooser.addOption("3 Loading", threeLoading());
    autoChooser.addOption("1 Balance", new OneBalance(autoBuilder));

    SmartDashboard.putData("Autonomous Mode", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static CommandBase none() {
    return Commands.none();
  }

  public static CommandBase twoPlusBalanceLoading() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new TwoPlusBalanceLoading(autoBuilder, false),
        new TwoPlusBalanceLoading(autoBuilder, true));
  }

  public static CommandBase twoPlusBalanceWire() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new TwoPlusBalanceWire(autoBuilder, false), new TwoPlusBalanceWire(autoBuilder, true));
  }

  public static CommandBase threeWire() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new ThreePieceWire(autoBuilder, false), new ThreePieceWire(autoBuilder, true));
  }

  public static CommandBase threeLoading() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new ThreeLoading(autoBuilder, false), new ThreeLoading(autoBuilder, true));
  }

  private static HashMap<String, Command> buildEventMap() {
    return new HashMap<>(
        Map.ofEntries(
            Map.entry("outtake", RobotContainer.intake.outtakeCommand()),
            Map.entry(
                "preAimMidConeLoading",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0, 24.5)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-23),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 24.5)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(23),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 24.5))))),
            Map.entry(
                "preAimMidConeLoading2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.75, 24.5)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-23),
                                RobotContainer.pinkArm.setArmPositionCommand(0.74, 24.5)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(23),
                                RobotContainer.pinkArm.setArmPositionCommand(0.74, 24.5))))),
            Map.entry(
                "preAimMidCubeLoading",
                Commands.parallel(
                    RobotContainer.turret.setTurretAngleCommand(0),
                    RobotContainer.pinkArm.setArmPositionCommand(0, 19))),
            Map.entry(
                "preAimMidConeWire",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0, 25)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(24),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 25)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-24),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 25))))),
            Map.entry(
                "preAimHighCube",
                Commands.parallel(
                    RobotContainer.turret.setTurretAngleCommand(0),
                    RobotContainer.pinkArm.setArmPositionCommand(0, 30))),
            Map.entry("armToSafePos", new ArmToSafePos()),
            Map.entry(
                "armToZero",
                Commands.parallel(
                    RobotContainer.pinkArm.setArmPositionCommand(0, -10),
                    RobotContainer.turret.setTurretAngleCommand(0))),
            Map.entry(
                "cheeseCube",
                RobotContainer.intake
                    .setIntakeSpeedCommand(-1.0)
                    .withTimeout(0.5)
                    .andThen(RobotContainer.intake.stopIntakeCommand())),
            Map.entry(
                "intakeBehind",
                new ArmToFloorIntakeBehindPos()
                    .alongWith(
                        RobotContainer.jaw
                            .setJawStateCommand(Jaw.State.FLOOR_BEHIND_PICKUP)
                            .andThen(RobotContainer.intake.intakeCommand()))),
            Map.entry(
                "intakeFront",
                Commands.sequence(
                    Commands.runOnce(
                        () -> RobotContainer.setIntakeMode(Constants.IntakeMode.FLOOR)),
                    new SmartIntake(false))),
            Map.entry(
                "preAimHighCubeBehind",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.0, 155)
                    .alongWith(RobotContainer.turret.setTurretAngleCommand(0))),
            Map.entry(
                "preAimHighCubeBehind2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.4, 155)
                    .alongWith(AutoBuildingBlocks.aimTurretToHighCubeLL())),
            Map.entry("aimAtGamePiece", AutoBuildingBlocks.aimAtGamePiece()),
            Map.entry(
                "preAimMidCubeBehind", RobotContainer.pinkArm.setArmPositionCommand(0.0, 162)),
            Map.entry(
                "preAimMidCubeBehind2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.0, 162)
                    .alongWith(AutoBuildingBlocks.aimTurretToMidCubeLL())),
            Map.entry("intakeConeSlow", RobotContainer.intake.intakeCommand())));
  }
}
