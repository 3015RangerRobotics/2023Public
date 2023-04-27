package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.CycleTracker;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();
  public static final Intake intake = new Intake();
  public static final Jaw jaw = new Jaw();
  public static final PinkArm pinkArm = new PinkArm();
  public static final Turret turret = new Turret();
  //  public static final StealyWheely stealer = new StealyWheely();
  public static final Manhattan manhattan = new Manhattan();
  public static final LEDStrip leds = new LEDStrip();

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  //  public static final JetsonClient jetson = new JetsonClient();

  public static void init() {
    configureButtonBindings();

    SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheckCommand());
    SmartDashboard.putData("PDH", powerDistribution);

    pinkArm.setDefaultCommand(new PinkArmHoldPosition());
    turret.setDefaultCommand(new TurretHoldPosition());
    swerve.setDefaultCommand(new SwerveDriveWithGamepad());
  }

  private static void configureButtonBindings() {
    driver.START().whileTrue(new UntipRobot());
    driver
        .LT()
        .whileTrue(
            jaw.setJawStateCommand(Jaw.State.SCORING_POS)
                .alongWith(new ArmToScoringTrackerPos(true)))
        .onFalse(
            new ArmToSafePos(true)
                .alongWith(Commands.waitSeconds(1.0).andThen(intake.intakeHoldCommand())));
    driver
        .LB()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(false);
                      setIntakeMode(Constants.IntakeMode.SHELF);
                    })
                .andThen(new SmartIntake(false)))
        .onFalse(
            pinkArm
                .setArmPositionCommand(
                    Constants.PinkArm.Extension.extensionLengthShelf,
                    Constants.PinkArm.Joint.jointAngleShelf + 20)
                .withTimeout(0.5)
                .andThen(
                    new ArmToSafePos()
                        .alongWith(
                            Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff())))));
    driver
        .RB()
        .onTrue(
            intake
                .outtakeCommand()
                .withTimeout(1.0)
                .andThen(intake.stopIntakeCommand())
                .alongWith(Commands.runOnce(CycleTracker::trackCycle)));
    driver
        .RT()
        .and(driver.LT().negate())
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(true);
                      setIntakeMode(Constants.IntakeMode.FLOOR);
                    })
                .andThen(new SmartIntake(true).alongWith(new SwerveDriveWithGamepad(true))))
        .onFalse(
            new ArmToSafePos()
                .alongWith(
                    intake.intakeHoldCommand(),
                    jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));

    driver.B()
        .whileTrue(intake.intakeCommand().alongWith(new ArmToSafePos()))
        .onFalse(
            intake
                .intakeHoldCommand()
                .alongWith(
                    new ArmToSafePos(),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));
    driver.X().whileTrue(Commands.run(swerve::lockModules, swerve));

    driver.Y()
        .whileTrue(
            pinkArm
                .setArmPositionCommand(0, 90)
                .alongWith(
                    turret.setTurretAngleCommand(0),
                    Commands.sequence(manhattan.stopArm(), manhattan.setTankSpeed(0))));
    driver.A()
        .and(driver.LS())
        .debounce(0.25)
        .onTrue(
            Commands.waitUntil(
                    () -> DriverStation.getMatchTime() == -1 || DriverStation.getMatchTime() >= 4.0)
                .andThen(new ManhattanClimb()));

    driver
        .RS()
        .onTrue(
            Commands.run(
                    () -> {
                      if (!isCubeMode()) {
                        intake.setSpeed(0.1);
                      }
                    },
                    intake)
                .withTimeout(0.1)
                .andThen(intake.intakeHoldCommand()));

    // Driver Paddles: top left = DUp, bottom left = DLeft, top right = DRight, bottom right = DDown
    driver.DUp()
        .and(driver.LT().negate())
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(false);
                      setIntakeMode(Constants.IntakeMode.FLOOR);
                    })
                .andThen(new SmartIntake(false).alongWith(new SwerveDriveWithGamepad(true))))
        .onFalse(
            new ArmToSafePos()
                .alongWith(
                    intake.intakeHoldCommand(),
                    jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));
    driver.DLeft()
        .and(driver.LT().negate())
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(false);
                      setIntakeMode(Constants.IntakeMode.FLOOR);
                    })
                .andThen(new SmartIntake(true).alongWith(new SwerveDriveWithGamepad(true))))
        .onFalse(
            new ArmToSafePos()
                .alongWith(
                    intake.intakeHoldCommand(),
                    jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));
    driver.DRight()
        .and(driver.LT().negate())
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(false);
                      setIntakeMode(Constants.IntakeMode.CHUTE);
                    })
                .andThen(new SmartIntake(false)))
        .onFalse(
            new ArmToSafePos()
                .alongWith(
                    intake.intakeHoldCommand(),
                    jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));
    driver.DDown()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      setIsCubeMode(true);
                      setIntakeMode(Constants.IntakeMode.CHUTE);
                    })
                .andThen(new SmartIntake(true)))
        .onFalse(
            new ArmToSafePos()
                .alongWith(
                    intake.intakeHoldCommand(),
                    jaw.setJawStateCommand(Jaw.State.SCORING_POS),
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff()))));

    coDriver.Y().whileTrue(new ScoreHighManual()).onFalse(new ArmToSafePos());
    coDriver.B().whileTrue(new ScoreMidManual()).onFalse(new ArmToSafePos());
    coDriver.A().whileTrue(new ScoreLowManual()).onFalse(new ArmToSafePos());

    coDriver
        .START()
        .and(coDriver.BACK())
        .onTrue(
            Commands.parallel(
                turret.angleManualControl(coDriver::getRightX),
                pinkArm.armManualControlCommand(
                    coDriver::getRightY, coDriver.LT(), coDriver.RT())));
  }

  /**
   * This method creates a command group for running all system check commands
   *
   * @return The full-robot system check command
   */
  public static CommandBase allSystemsCheckCommand() {
    return Commands.sequence(
        pinkArm.getSystemCheckCommand(),
        turret.getSystemCheckCommand(),
        swerve.getSystemCheckCommand(),
        jaw.getSystemCheckCommand(),
        intake.getSystemCheckCommand(),
        //        stealer.getSystemCheckCommand(),
        manhattan.getSystemCheckCommand(),
        Commands.runOnce(
            () -> {
              if (allSystemsOK()) {
                leds.setState(LEDStrip.State.PARTY);
              } else {
                leds.setState(LEDStrip.State.SAD);
              }
            }),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> leds.setState(LEDStrip.State.NORMAL)));
  }

  public static boolean allSystemsOK() {
    return pinkArm.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && turret.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && swerve.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && jaw.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && intake.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        //        && stealer.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && manhattan.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK;
  }

  public static boolean isCubeMode() {
    return SmartDashboard.getBoolean("CubeMode", true);
  }

  public static Constants.IntakeMode getIntakeMode() {
    String mode = SmartDashboard.getString("IntakeMode", "floor");

    if (mode.equals("shelf")) {
      return Constants.IntakeMode.SHELF;
    } else if (mode.equals("chute")) {
      return Constants.IntakeMode.CHUTE;
    } else {
      return Constants.IntakeMode.FLOOR;
    }
  }

  public static void setIntakeMode(Constants.IntakeMode intakeMode) {
    String modeStr = "";
    switch (intakeMode) {
      case CHUTE:
        modeStr = "chute";
        break;
      case FLOOR:
        modeStr = "floor";
        break;
      case SHELF:
        modeStr = "shelf";
        break;
    }
    SmartDashboard.putString("IntakeMode", modeStr);
  }

  public static void setIsCubeMode(boolean cubeMode) {
    SmartDashboard.putBoolean("CubeMode", cubeMode);
  }

  public static boolean isPoleLLEnabled() {
    return SmartDashboard.getBoolean("PoleLLEnabled", true);
  }

  public static boolean isPickupLLEnabled() {
    return SmartDashboard.getBoolean("PickupLLEnabled", true);
  }
}
