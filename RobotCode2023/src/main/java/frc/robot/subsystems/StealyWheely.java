package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class StealyWheely extends AdvancedSubsystem {
  private final TalonSRX intakeWheels;

  public StealyWheely() {
    intakeWheels = new TalonSRX(Constants.StealyWheely.motorID);

    intakeWheels.configFactoryDefault();
    intakeWheels.setInverted(false);
    intakeWheels.configPeakCurrentLimit(Constants.StealyWheely.motorCurrentLimit);
    intakeWheels.configContinuousCurrentLimit(Constants.StealyWheely.motorCurrentLimit);
    intakeWheels.configVoltageCompSaturation(12.0);
    intakeWheels.enableVoltageCompensation(true);

    registerHardware("Intake Wheels", intakeWheels);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public void setIntakeWheelsSpeed(double percentOutput) {
    intakeWheels.set(ControlMode.PercentOutput, percentOutput);
  }

  public CommandBase intakeCommand() {
    return Commands.run(() -> setIntakeWheelsSpeed(Constants.StealyWheely.intakeSpeed), this);
  }

  public CommandBase outtakeCommand() {
    return Commands.run(() -> setIntakeWheelsSpeed(Constants.StealyWheely.outtakeSpeed), this);
  }

  public CommandBase intakeHoldCommand() {
    return Commands.run(() -> setIntakeWheelsSpeed(Constants.StealyWheely.intakeHoldSpeed), this);
  }

  public CommandBase stopIntakeWheelsCommand() {
    return Commands.runOnce(() -> setIntakeWheelsSpeed(0.0), this);
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> setIntakeWheelsSpeed(1.0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (intakeWheels.getSupplyCurrent() < 1) {
                    addFault(
                        "[System Check] Stealer wheels current forward current draw too low",
                        false,
                        true);
                  }
                  if (intakeWheels.getSupplyCurrent() > 30) {
                    addFault(
                        "[System Check] Stealer wheels current forward current draw too high",
                        false,
                        true);
                  }
                },
                this),
            Commands.runOnce(() -> setIntakeWheelsSpeed(-1.0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (intakeWheels.getSupplyCurrent() < 1) {
                    addFault(
                        "[System Check] Stealer wheels current reverse current draw too low",
                        false,
                        true);
                  }
                  if (intakeWheels.getSupplyCurrent() > 30) {
                    addFault(
                        "[System Check] Stealer wheels current reverse current draw too high",
                        false,
                        true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> setIntakeWheelsSpeed(0.0), this));
  }
}
