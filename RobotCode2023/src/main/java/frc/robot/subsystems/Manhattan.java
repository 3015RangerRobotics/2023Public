package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.*;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Manhattan extends AdvancedSubsystem {
  protected final TalonFX armMotor;
  protected final CANcoder armEncoder;
  protected final TalonSRX tankMotor;

  private final StatusSignalValue<Double> armTempSignal;
  private final StatusSignalValue<Double> armPosSignal;

  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  public Manhattan() {
    armEncoder = new CANcoder(Constants.Manhattan.armEncoderID, Constants.canivoreBusName);
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble("ManhattanEncoderOffset", 0.0) / 360.0;
    armEncoder.getConfigurator().apply(encoderConfig);

    armMotor = new TalonFX(Constants.Manhattan.armID, Constants.canivoreBusName);
    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    armConfig.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
    armMotor.getConfigurator().apply(armConfig);

    armTempSignal = armMotor.getDeviceTemp();
    armPosSignal = armEncoder.getAbsolutePosition();

    tankMotor = new TalonSRX(Constants.Manhattan.tankID);
    tankMotor.configPeakCurrentLimit(Constants.Manhattan.tankCurrentLimit);
    tankMotor.configContinuousCurrentLimit(Constants.Manhattan.tankCurrentLimit);
    tankMotor.enableVoltageCompensation(true);
    tankMotor.configVoltageCompSaturation(12.0);
    tankMotor.setInverted(true);

    registerHardware("Arm Motor", armMotor);
    registerHardware("Tank Motor", tankMotor);
    registerHardware("Arm Encoder", armEncoder);

    SmartDashboard.putData(
        "Manhattan/TrimArmsEncoder",
        Commands.runOnce(this::updateArmEncoderOffset, this).ignoringDisable(true));
    SmartDashboard.putData("Manhattan/ArmsToZero", armsToZero());
    SmartDashboard.putData(
        "Manhattan/ResetArns",
        armsToZero().andThen(setArmVoltage(-2.0).withTimeout(2.0), armsToZero()));

    setDefaultCommand(
        Commands.run(
            () -> {
              tankMotor.set(ControlMode.PercentOutput, 0.0);
              armMotor.setControl(new VoltageOut(0.0));
            },
            this));
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    SmartDashboard.putNumber("Manhattan/ArmTemp", armTempSignal.getValue());
    SmartDashboard.putNumber("Manhattan/ArmAngle", getArmAngle());

    StatusSignalValue.waitForAll(0.0, armPosSignal, armTempSignal);

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Manhattan/PeriodicRuntime", runtimeMS);
  }

  public double getArmAngle() {
    return armPosSignal.getValue() * 360.0;
  }

  public CommandBase setTankSpeed(double percentOutput) {
    return Commands.runOnce(() -> tankMotor.set(ControlMode.PercentOutput, percentOutput));
  }

  public CommandBase armsToZero() {
    return Commands.run(
            () -> {
              if (getArmAngle() > 0) {
                armMotor.setControl(new VoltageOut(-2.0));
              } else {
                armMotor.setControl(new VoltageOut(2.0));
              }
            },
            this)
        .until(() -> Math.abs(getArmAngle() - 0) <= 1.0)
        .andThen(setArmVoltage(0.0).withTimeout(0.1));
  }

  public CommandBase setArmVoltage(double voltage) {
    return Commands.run(() -> armMotor.setControl(new VoltageOut(voltage)), this);
  }

  public CommandBase stopArm() {
    return Commands.runOnce(() -> armMotor.setControl(new VoltageOut(0.0)), this);
  }

  public void updateArmEncoderOffset() {
    double currentOffset = encoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - armPosSignal.getValue()) % 1.0;
    Preferences.setDouble("ManhattanEncoderOffset", offset * 360.0);
    encoderConfig.MagnetSensor.MagnetOffset = offset;
    armEncoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> armMotor.setControl(new VoltageOut(-2.0)), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  //                  if (armEncoder.getVelocity().getValue() == 0.0) {
                  //                    addFault("[System Check] Arms encoder velocity measured too
                  // slow", false, true);
                  //                  }

                  armMotor.setControl(new VoltageOut(0.0));
                },
                this),
            setArmVoltage(8.0).until(() -> getArmAngle() >= 50).withTimeout(5.0),
            Commands.runOnce(
                () -> {
                  if (getArmAngle() < 50) {
                    addFault("[System Check] Arms did not reach target position");
                  }

                  armMotor.setControl(new VoltageOut(0.0));
                },
                this),
            armsToZero().withTimeout(5.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getArmAngle()) > 3) {
                    addFault("[System Check] Arms did not reach target position");
                  }

                  armMotor.setControl(new VoltageOut(0.0));
                },
                this),
            setTankSpeed(1.0),
            Commands.waitSeconds(3.0),
            Commands.runOnce(
                () -> {
                  if (tankMotor.getSupplyCurrent() < 1) {
                    addFault("[System Check] Tank tread current draw too low");
                  }

                  tankMotor.set(ControlMode.PercentOutput, 0.0);
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(
            Commands.runOnce(
                () -> {
                  armMotor.stopMotor();
                  tankMotor.set(ControlMode.PercentOutput, 0.0);
                },
                this));
  }
}
