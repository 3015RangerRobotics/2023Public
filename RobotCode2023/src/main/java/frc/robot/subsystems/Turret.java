package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class Turret extends AdvancedSubsystem {
  protected final TalonFX turretMotor;
  protected final TalonFXSimState turretMotorSim;
  protected final StatusSignalValue<Double> turretErrorSignal;
  protected final StatusSignalValue<Double> turretCurrentSignal;
  protected final StatusSignalValue<Double> turretVoltageSignal;
  protected final StatusSignalValue<Double> turretTempSignal;

  protected final CANcoder turretEncoder;
  protected final CANcoderSimState turretEncoderSim;
  protected final CANcoderConfiguration turretEncoderConfig;
  protected final StatusSignalValue<Double> turretAbsolutePosSignal;
  protected final StatusSignalValue<Double> turretAbsoluteVelSignal;

  protected final LinearSystemSim<N2, N1, N1> turretSim;

  public double turretSetpointDegrees = 0.0;

  //  private final Mechanism2d turretMech = new Mechanism2d(1.0, 1.0, new Color8Bit(27, 27, 31));
  //  private final MechanismRoot2d turretRoot = turretMech.getRoot("turretRoot", 0.5, 0.5);
  //  private final MechanismLigament2d turretTriBaseR =
  //      turretRoot.append(
  //          new MechanismLigament2d("turretTriBaseR", 0.1, 0, 6, new Color8Bit(238, 238, 238)));
  //  private final MechanismLigament2d turretTriBaseL =
  //      turretTriBaseR.append(
  //          new MechanismLigament2d("turretTriBaseL", 0.2, 180, 6, new Color8Bit(238, 238, 238)));
  //  private final MechanismLigament2d turretTriR =
  //      turretTriBaseR.append(
  //          new MechanismLigament2d("turretTriR", 0.4, 104, 6, new Color8Bit(238, 238, 238)));
  //  private final MechanismLigament2d turretTriL =
  //      turretTriBaseL.append(
  //          new MechanismLigament2d("turretTriL", 0.4, -104, 6, new Color8Bit(238, 238, 238)));

  public Turret() {
    turretMotor = new TalonFX(Constants.Turret.motorID, Constants.canivoreBusName);
    turretEncoder = new CANcoder(Constants.Turret.encoderID, Constants.canivoreBusName);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    turretEncoderConfig = new CANcoderConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Turret.maxVel / Constants.Turret.degreesPerRotation;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        Constants.Turret.maxAccel / Constants.Turret.degreesPerRotation;
    motorConfig.MotionMagic.MotionMagicJerk =
        Constants.Turret.maxJerk / Constants.Turret.degreesPerRotation;
    motorConfig.Slot0.kP = Constants.Turret.kP;
    motorConfig.Slot0.kI = Constants.Turret.kI;
    motorConfig.Slot0.kD = Constants.Turret.kD;
    motorConfig.Slot0.kV = Constants.Turret.kV;
    motorConfig.Slot0.kS = Constants.Turret.kS;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Turret.FORWARD_LIMIT / Constants.Turret.degreesPerRotation;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Turret.REVERSE_LIMIT / Constants.Turret.degreesPerRotation;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();

    turretEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turretEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    turretEncoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble("TurretEncoderOffset", 0.0) / 360.0;

    turretMotor.getConfigurator().apply(motorConfig);
    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    turretMotorSim = turretMotor.getSimState();
    turretEncoderSim = turretEncoder.getSimState();

    turretSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(
                Constants.Turret.SimInfo.kV, Constants.Turret.SimInfo.kA));

    turretErrorSignal = turretMotor.getClosedLoopError();
    turretCurrentSignal = turretMotor.getSupplyCurrent();
    turretVoltageSignal = turretMotor.getSupplyVoltage();
    turretTempSignal = turretMotor.getDeviceTemp();

    turretAbsolutePosSignal = turretEncoder.getAbsolutePosition();
    turretAbsoluteVelSignal = turretEncoder.getVelocity();

    //    SmartDashboard.putData("Turret/Mech2d", turretMech);
    SmartDashboard.putData(
        "Turret/TrimTurretEncoder",
        Commands.runOnce(this::updateTurretEncoderOffset, this).ignoringDisable(true));

    registerHardware("Turret Motor", turretMotor);
    registerHardware("Turret Encoder", turretEncoder);
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    //    turretTriBaseR.setAngle(getTurretAngle());

    SmartDashboard.putNumber("Turret/ActualAngle", getTurretAngle());
    SmartDashboard.putNumber("Turret/TurretTemp", turretTempSignal.getValue());

    StatusSignalValue.waitForAll(
        0,
        turretErrorSignal,
        turretCurrentSignal,
        turretVoltageSignal,
        turretTempSignal,
        turretAbsolutePosSignal,
        turretAbsoluteVelSignal);

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Turret/PeriodicRuntime", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {
    turretMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    turretSim.setInput(turretMotorSim.getMotorVoltage());

    turretSim.update(0.02);

    double turretPos = turretSim.getOutput(0);
    double turretDeltaPos = turretPos - getTurretAngle();
    turretMotorSim.setRawRotorPosition(turretPos / Constants.Turret.degreesPerRotation);
    turretMotorSim.setRotorVelocity(turretDeltaPos / 0.02 / Constants.Turret.degreesPerRotation);
    turretEncoderSim.setRawPosition(turretPos / 360.0);
    turretEncoderSim.setVelocity(turretDeltaPos / 0.02 / 360.0);
  }

  /**
   * Set the desired angle of the turret
   *
   * @param rotation The desired rotation in degrees
   */
  public void setDesiredAngle(double rotation) {
    turretSetpointDegrees = rotation;
    double targetAngle =
        Math.min(
            Constants.Turret.FORWARD_LIMIT, Math.max(Constants.Turret.REVERSE_LIMIT, rotation));

    turretMotor.setControl(
        new MotionMagicVoltage(targetAngle / Constants.Turret.degreesPerRotation));
  }

  public void setTurretVoltage(double voltage) {
    turretMotor.setControl(new VoltageOut(voltage));
  }

  /** Update the absolute encoder offset. The current position will become the new zero */
  public void updateTurretEncoderOffset() {
    double currentOffset = turretEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - turretAbsolutePosSignal.getValue()) % 1.0;
    Preferences.setDouble("TurretEncoderOffset", offset * 360.0);
    turretEncoderConfig.MagnetSensor.MagnetOffset = offset;
    turretEncoder.getConfigurator().apply(turretEncoderConfig);
  }

  /**
   * Get the value of the absolute encoder
   *
   * @return Absolute turret angle in degrees
   */
  public double getTurretAngle() {
    return turretAbsolutePosSignal.getValue() * 360.0;
  }

  /**
   * Create a command to set the turret angle
   *
   * @param turretAngle Desired turret angle in degrees
   * @return Command to rotate turret to given angle
   */
  public CommandBase setTurretAngleCommand(double turretAngle) {
    return Commands.run(() -> setDesiredAngle(turretAngle), this);
  }

  public void pointTurretToPosition(Translation3d targetPos, double aimPointLeft) {
    double aimPointForward =
        RobotContainer.isCubeMode()
            ? Constants.PointToPosition.cubeForward
            : Constants.PointToPosition.coneForward;

    Pose2d robotPose = RobotContainer.swerve.getPose();
    Rotation2d jointToTargetAngle =
        targetPos.toTranslation2d().minus(robotPose.getTranslation()).getAngle();

    double distance =
        targetPos.toTranslation2d().getDistance(robotPose.getTranslation()) + aimPointForward;

    double cosC =
        (Math.pow(distance, 2) + Math.pow(distance, 2) - Math.pow(aimPointLeft, 2))
            / (2 * distance * distance);
    double angleOffset = Math.copySign(Math.acos(cosC), aimPointLeft);

    setDesiredAngleFieldRelative(jointToTargetAngle.minus(new Rotation2d(angleOffset)), robotPose);
  }

  public double getTurretVelocity() {
    return turretAbsoluteVelSignal.getValue() * 360.0;
  }

  public CommandBase pointTurretToPositionCommand(Translation3d fieldPos, double aimPointLeft) {
    return Commands.run(() -> pointTurretToPosition(fieldPos, aimPointLeft), this);
  }

  /**
   * Create a command to move the turret to the resting position
   *
   * @return Command to move turret to resting position
   */
  public CommandBase turretRestingCommand() {
    return Commands.run(() -> setDesiredAngle(Constants.Turret.restingAngle), this);
  }

  /**
   * Set the desired angle of the turret to a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to set turret to
   * @param currentPose Current pose of the robot
   */
  public void setDesiredAngleFieldRelative(Rotation2d fieldRelativeAngle, Pose2d currentPose) {
    double desiredAngle = fieldRelativeAngle.minus(currentPose.getRotation()).getDegrees();

    if (desiredAngle < Constants.Turret.FORWARD_LIMIT
        && desiredAngle > Constants.Turret.REVERSE_LIMIT) {
      setDesiredAngle(desiredAngle);
    } else {
      setDesiredAngle(Constants.Turret.restingAngle);
    }
  }

  /**
   * Create a command to hold the turret at a field relative angle
   *
   * @param fieldRelativeAngle The field relative angle to hold, in degrees
   * @return Command to hold turret at field relative angle
   */
  public CommandBase fieldRelativeAngleCommand(double fieldRelativeAngle) {
    return Commands.run(
        () ->
            setDesiredAngleFieldRelative(
                Rotation2d.fromDegrees(fieldRelativeAngle), RobotContainer.swerve.getPose()),
        this);
  }

  /**
   * Create a command to move the turret angle manually
   *
   * @param angleControl gets the button configured to command
   * @return A command to move the turret angle manually
   */
  public CommandBase angleManualControl(DoubleSupplier angleControl) {
    return Commands.run(
        () ->
            setDesiredAngle(
                getTurretAngle() + angleControl.getAsDouble() * Constants.Turret.incrementAngle),
        this);
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> turretMotor.setControl(new DutyCycleOut(0.3)), this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (turretAbsoluteVelSignal.getValue() * 360.0 < 5) {
                    addFault(
                        "[System Check] Turret absolute encoder velocity measured too slow",
                        false,
                        true);
                  }
                  turretMotor.setControl(new StaticBrake());
                },
                this),
            Commands.runOnce(() -> setDesiredAngle(-20.0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getTurretAngle() < -25 || getTurretAngle() > -15) {
                    addFault("[System Check] Turret did not reach desired angle", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setDesiredAngle(0.0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getTurretAngle()) > 5) {
                    addFault("[System Check] Turret did not reach desired angle", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> turretMotor.setControl(new StaticBrake()), this));
  }
}
