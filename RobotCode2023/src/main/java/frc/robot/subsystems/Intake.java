package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleOn;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.IntakeCam;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.ScoringTracker;
import java.util.Optional;

public class Intake extends AdvancedSubsystem {
  protected final CANSparkMax intakeWheelMotor;

  public Intake() {
    intakeWheelMotor =
        new CANSparkMax(Constants.Intake.intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    intakeWheelMotor.restoreFactoryDefaults();
    intakeWheelMotor.setSmartCurrentLimit(Constants.Intake.intakeCurrentLimit);
    intakeWheelMotor.enableVoltageCompensation(12.0);
    intakeWheelMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    intakeWheelMotor.getEncoder().setMeasurementPeriod(10);

    registerHardware("Intake Motor", intakeWheelMotor);

    Robot.addPeriodicCallback(
        () -> {
          if (RobotBase.isReal() && DriverStation.isDisabled() && !IntakeCam.isCameraConnected()) {
            addFault("Intake Cam not connected");
          }
        },
        5.0);
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("Intake/IntakeTemp", intakeWheelMotor.getMotorTemperature());

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Intake/PeriodicRuntime", runtimeMS);
  }

  public double getIntakeMotorRPM() {
    return intakeWheelMotor.getEncoder().getVelocity();
  }

  /**
   * Gets the current output voltage to the intake roller motor
   *
   * @return Current intake wheel output voltage
   */
  public double getWheelVoltage() {
    return intakeWheelMotor.getBusVoltage();
  }

  /**
   * Sets the speed of the roller wheel
   *
   * @param percentOutput Percent speed
   */
  public void setSpeed(double percentOutput) {
    intakeWheelMotor.set(percentOutput);
  }

  /**
   * Create a command to set the intake speed
   *
   * @param percentOutput Percent speed
   * @return Command to set intake speed
   */
  public CommandBase setIntakeSpeedCommand(double percentOutput) {
    return Commands.run(() -> setSpeed(percentOutput), this);
  }

  public CommandBase stopIntakeCommand() {
    return Commands.runOnce(() -> setSpeed(0), this);
  }

  public CommandBase intakeHoldCommand() {
    return Commands.run(
        () -> {
          if (RobotContainer.isCubeMode()) {
            setSpeed(Constants.Intake.intakeHoldPercentCube);
          } else {
            setSpeed(Constants.Intake.intakeHoldPercentCone);
          }
        },
        this);
  }

  /**
   * command to outtake a cube or cone based on the mode
   *
   * @return a command to outtake a cube or cone
   */
  public CommandBase outtakeCommand() {
    return Commands.run(
        () -> {
          if (RobotContainer.isCubeMode()) {
            Optional<Integer> scoringTrackerOverride = ScoringTracker.getScoringTrackerOverride();
            if (scoringTrackerOverride.isPresent() && scoringTrackerOverride.get() >= 18) {
              setSpeed(Constants.Intake.outtakeSpeedCubeLow);
            } else {
              setSpeed(Constants.Intake.outtakeSpeedCube);
            }
          } else {
            setSpeed(Constants.Intake.outtakeSpeedCone);
          }
        },
        this);
  }

  /**
   * command to intake a cube or cone based on the mode
   *
   * @return a command to intake a cube or cone
   */
  public CommandBase intakeCommand() {
    return Commands.run(
        () -> {
          if (RobotContainer.isCubeMode()) {
            setSpeed(Constants.Intake.intakeSpeedCube);
          } else {
            setSpeed(Constants.Intake.intakeSpeedCone);
          }

          if (!DriverStation.isAutonomousEnabled()) {
            if (Math.abs(getIntakeMotorRPM()) < 60) {
              RobotContainer.driver.setRumbleAnimation(new RumbleOn());
            } else {
              RobotContainer.driver.setRumbleAnimation(new RumbleOff());
            }
          }
        },
        this);
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (RobotBase.isReal() && !IntakeCam.isCameraConnected()) {
                    addFault(
                        "[System Check] Intake camera not functional, sink timed out", false, true);
                  }

                  intakeWheelMotor.set(1.0);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(intakeWheelMotor.getEncoder().getVelocity()) < 0.1) {
                    addFault("[System Check] Intake motor not moving", false, true);
                  }
                  intakeWheelMotor.set(-1.0);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(intakeWheelMotor.getEncoder().getVelocity()) < 0.1) {
                    addFault("[System Check] Intake motor not moving", false, true);
                  }
                  intakeWheelMotor.set(0.0);
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(stopIntakeCommand());
  }
}
