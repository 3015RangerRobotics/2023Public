package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PinkArm extends AdvancedSubsystem {
  protected final TalonFX extensionMotor;
  protected final TalonFXSimState extensionMotorSim;
  protected final StatusSignalValue<Double> extensionCurrentSignal;
  protected final StatusSignalValue<Double> extensionVoltageSignal;
  protected final StatusSignalValue<Double> extensionErrorSignal;
  protected final StatusSignalValue<Double> extensionPosSignal;
  protected final StatusSignalValue<Double> extensionVelSignal;
  protected final StatusSignalValue<Double> extensionTempSignal;

  protected final TalonFX jointMotor;
  protected final TalonFXSimState jointMotorSim;
  protected final StatusSignalValue<Double> jointCurrentSignal;
  protected final StatusSignalValue<Double> jointVoltageSignal;
  protected final StatusSignalValue<Double> jointErrorSignal;
  protected final StatusSignalValue<Double> jointTempSignal;

  protected final CANcoder jointEncoder;
  protected final CANcoderSimState jointEncoderSim;
  protected final CANcoderConfiguration jointEncoderConfig;
  protected final StatusSignalValue<Double> jointAbsolutePosSignal;
  protected final StatusSignalValue<Double> jointAbsoluteVelSignal;

  protected final SingleJointedArmSim jointSim;
  protected final LinearSystemSim<N2, N1, N1> extensionSim;

  public double extensionSetpoint = 0;
  public double jointSetpoint = 0;

  //  private final Mechanism2d pinkArmMech = new Mechanism2d(2.5, 2.5, new Color8Bit(27, 27, 31));
  //
  //  private final MechanismRoot2d jointRoot = pinkArmMech.getRoot("jointRoot", 0.6, 0.5);
  //  private final MechanismLigament2d baseLeft =
  //      jointRoot.append(
  //          new MechanismLigament2d("baseLeft", 0.6, -120, 5, new Color8Bit(80, 80, 80)));
  //  private final MechanismLigament2d baseRight =
  //      jointRoot.append(
  //          new MechanismLigament2d("baseRight", 0.6, -60, 5, new Color8Bit(80, 80, 80)));
  //  private final MechanismLigament2d proximalSegmentForward =
  //      jointRoot.append(
  //          new MechanismLigament2d(
  //              "proximalSegmentForward", 0.5, 0, 16, new Color8Bit(238, 238, 238)));
  //  private final MechanismLigament2d proximalSegmentBackward =
  //      proximalSegmentForward.append(
  //          new MechanismLigament2d(
  //              "proximalSegmentBackward", 1.0, 180, 16, new Color8Bit(238, 238, 238)));
  //  private final MechanismLigament2d midSegment =
  //      proximalSegmentForward.append(
  //          new MechanismLigament2d("midSegment", 0.05, 0, 10, new Color8Bit(180, 180, 180)));
  //  private final MechanismLigament2d distalSegment =
  //      midSegment.append(
  //          new MechanismLigament2d("distalSegment", 0.05, 0, 5, new Color8Bit(120, 120, 120)));

  public PinkArm() {
    extensionMotor = new TalonFX(Constants.PinkArm.Extension.motorID, Constants.canivoreBusName);
    jointMotor = new TalonFX(Constants.PinkArm.Joint.motorID, Constants.canivoreBusName);
    jointEncoder = new CANcoder(Constants.PinkArm.Joint.encoderID, Constants.canivoreBusName);

    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    TalonFXConfiguration jointConfig = new TalonFXConfiguration();
    jointEncoderConfig = new CANcoderConfiguration();

    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.PinkArm.Extension.maxVel / Constants.PinkArm.Extension.metersPerRotation;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
        Constants.PinkArm.Extension.maxAccel / Constants.PinkArm.Extension.metersPerRotation;
    extensionConfig.MotionMagic.MotionMagicJerk =
        Constants.PinkArm.Extension.maxJerk / Constants.PinkArm.Extension.metersPerRotation;
    extensionConfig.Slot0.kP = Constants.PinkArm.Extension.kP;
    extensionConfig.Slot0.kI = Constants.PinkArm.Extension.kI;
    extensionConfig.Slot0.kD = Constants.PinkArm.Extension.kD;
    extensionConfig.Slot0.kV = Constants.PinkArm.Extension.kV;
    extensionConfig.Slot0.kS = Constants.PinkArm.Extension.kS;

    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.PinkArm.Extension.FORWARD_LIMIT / Constants.PinkArm.Extension.metersPerRotation;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.PinkArm.Extension.REVERSE_LIMIT / Constants.PinkArm.Extension.metersPerRotation;

    jointConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.PinkArm.Joint.maxVel / Constants.PinkArm.Joint.degreesPerRotation;
    jointConfig.MotionMagic.MotionMagicAcceleration =
        Constants.PinkArm.Joint.maxAccel / Constants.PinkArm.Joint.degreesPerRotation;
    jointConfig.MotionMagic.MotionMagicJerk =
        Constants.PinkArm.Joint.maxJerk / Constants.PinkArm.Joint.degreesPerRotation;
    jointConfig.Slot0.kP = Constants.PinkArm.Joint.kP;
    jointConfig.Slot0.kI = Constants.PinkArm.Joint.kI;
    jointConfig.Slot0.kD = Constants.PinkArm.Joint.kD;
    jointConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    jointConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.PinkArm.Joint.FORWARD_LIMIT / Constants.PinkArm.Joint.degreesPerRotation;
    jointConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    jointConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.PinkArm.Joint.REVERSE_LIMIT / Constants.PinkArm.Joint.degreesPerRotation;
    jointConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (RobotBase.isReal()) {
      jointConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      jointConfig.Feedback.FeedbackRemoteSensorID = jointEncoder.getDeviceID();
    }

    jointEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    jointEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    jointEncoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble("PinkArmJointEncoderOffset", 0.0) / 360.0;

    extensionMotor.getConfigurator().apply(extensionConfig);
    jointMotor.getConfigurator().apply(jointConfig);
    jointEncoder.getConfigurator().apply(jointEncoderConfig);

    extensionCurrentSignal = extensionMotor.getSupplyCurrent();
    extensionVoltageSignal = extensionMotor.getClosedLoopError();
    extensionErrorSignal = extensionMotor.getClosedLoopError();
    extensionPosSignal = extensionMotor.getPosition();
    extensionVelSignal = extensionMotor.getVelocity();
    extensionTempSignal = extensionMotor.getDeviceTemp();

    jointCurrentSignal = jointMotor.getSupplyCurrent();
    jointVoltageSignal = jointMotor.getSupplyVoltage();
    jointErrorSignal = jointMotor.getClosedLoopError();
    jointTempSignal = jointMotor.getDeviceTemp();

    jointAbsolutePosSignal = jointEncoder.getAbsolutePosition();
    jointAbsoluteVelSignal = jointEncoder.getVelocity();

    extensionMotorSim = extensionMotor.getSimState();
    jointMotorSim = jointMotor.getSimState();
    jointEncoderSim = jointEncoder.getSimState();

    jointSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500(1),
                Constants.PinkArm.Joint.SimInfo.MOI,
                1.0 / Constants.PinkArm.Joint.gearing),
            DCMotor.getFalcon500(1),
            1.0 / Constants.PinkArm.Joint.gearing,
            Constants.PinkArm.Joint.SimInfo.extendedJointToEndLength,
            Units.degreesToRadians(Constants.PinkArm.Joint.REVERSE_LIMIT + 90),
            Units.degreesToRadians(Constants.PinkArm.Joint.FORWARD_LIMIT + 90),
            false);

    extensionSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(
                Constants.PinkArm.Extension.SimInfo.kV, Constants.PinkArm.Extension.SimInfo.kA));

    //    SmartDashboard.putData("PinkArm/Mech2d", pinkArmMech);

    SmartDashboard.putData(
        "PinkArm/TrimJointEncoder",
        Commands.runOnce(this::updateJointEncoderOffset, this).ignoringDisable(true));

    if (RobotBase.isReal()) {
      jointMotor.setRotorPosition(getJointAngle() / Constants.PinkArm.Joint.degreesPerRotation);
      extensionMotor.setRotorPosition(0);
    }

    registerHardware("Extension Motor", extensionMotor);
    registerHardware("Joint Motor", jointMotor);
    registerHardware("Joint Encoder", jointEncoder);
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    //    proximalSegmentForward.setAngle(getJointAngle());

    //    midSegment.setLength(0.05 + (getExtensionLength() / 2.0));
    //    distalSegment.setLength(0.05 + (getExtensionLength() / 2.0));

    SmartDashboard.putNumber("PinkArm/ExtensionTemp", extensionMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber("PinkArm/JointTemp", jointMotor.getDeviceTemp().getValue());

    StatusSignalValue.waitForAll(
        0,
        extensionCurrentSignal,
        extensionVoltageSignal,
        extensionErrorSignal,
        extensionPosSignal,
        extensionVelSignal,
        extensionTempSignal,
        jointCurrentSignal,
        jointVoltageSignal,
        jointErrorSignal,
        jointTempSignal,
        jointAbsolutePosSignal,
        jointAbsoluteVelSignal);

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("PinkArm/PeriodicRuntime", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {
    extensionMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    jointMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    jointSim.setInputVoltage(jointMotorSim.getMotorVoltage());
    extensionSim.setInput(extensionMotorSim.getMotorVoltage());

    jointSim.update(0.02);
    extensionSim.update(0.02);

    double jointPos = Units.radiansToDegrees(jointSim.getAngleRads()) - 90;
    double jointVel = Units.radiansToDegrees(jointSim.getVelocityRadPerSec());
    jointMotorSim.setRawRotorPosition(jointPos / Constants.PinkArm.Joint.degreesPerRotation);
    jointMotorSim.setRotorVelocity(jointVel / Constants.PinkArm.Joint.degreesPerRotation);
    jointEncoderSim.setRawPosition(jointPos / 360.0);
    jointEncoderSim.setVelocity(jointVel / 360.0);

    double extensionPos = extensionSim.getOutput(0);
    double extensionDeltaPos = extensionPos - getExtensionLength();
    extensionMotorSim.setRawRotorPosition(
        extensionPos / Constants.PinkArm.Extension.metersPerRotation);
    extensionMotorSim.setRotorVelocity(
        extensionDeltaPos / 0.02 / Constants.PinkArm.Extension.metersPerRotation);
  }

  /**
   * Set the goal length of the arm extension
   *
   * @param length The goal extension in meters
   */
  public void setExtensionGoalLength(double length) {
    double targetLength =
        Math.min(
            Constants.PinkArm.Extension.FORWARD_LIMIT,
            Math.max(Constants.PinkArm.Extension.REVERSE_LIMIT, length));
    extensionSetpoint = targetLength;

    if (Math.abs(targetLength - getExtensionLength())
        < Constants.PinkArm.Extension.allowableExtensionError) {
      extensionMotor.setControl(new StaticBrake());
    } else {
      extensionMotor.setControl(
          new MotionMagicVoltage(targetLength / Constants.PinkArm.Extension.metersPerRotation));
    }
  }

  /**
   * Set the goal angle of the arm joint
   *
   * @param degrees Goal angle in degrees
   */
  public void setJointGoalAngle(double degrees) {
    double targetAngle = degrees - 90; // Given angle is 0 horizontal, mech angle is 0 pointing up
    jointSetpoint =
        Math.min(
            Constants.PinkArm.Joint.FORWARD_LIMIT,
            Math.max(Constants.PinkArm.Joint.REVERSE_LIMIT, targetAngle));

    jointMotor.setControl(
        new MotionMagicVoltage(jointSetpoint / Constants.PinkArm.Joint.degreesPerRotation));
  }

  /**
   * Set the goal position of the arm
   *
   * @param extensionLength Goal length of the extension in meters
   * @param jointAngle Goal angle of the joint in degrees
   */
  public void setArmGoalPosition(double extensionLength, double jointAngle) {
    setExtensionGoalLength(extensionLength);
    setJointGoalAngle(jointAngle);
  }

  /**
   * Get the length of the arm extension
   *
   * @return The extension in meters
   */
  public double getExtensionLength() {
    return extensionPosSignal.getValue() * Constants.PinkArm.Extension.metersPerRotation;
  }

  /**
   * Get the velocity of the arm extension
   *
   * @return Velocity of arm extension in m/s
   */
  public double getExtensionVelocity() {
    return extensionVelSignal.getValue() * Constants.PinkArm.Extension.metersPerRotation;
  }

  /**
   * Get the velocity of the arm joint
   *
   * @return Velocity of arm joint in deg/s
   */
  public double getJointVelocity() {
    return jointAbsoluteVelSignal.getValue() * 360.0;
  }

  /**
   * Create a command to set the position of the arm
   *
   * @param extensionLength The length of the extension in meters
   * @param jointAngle The angle of the joint in degrees
   * @return A command to move the arm to the given position
   */
  public CommandBase setArmPositionCommand(double extensionLength, double jointAngle) {
    return Commands.run(() -> setArmGoalPosition(extensionLength, jointAngle), this);
  }

  /**
   * Create a command to set the position of the arm to the resting position
   *
   * @return Command to move the arm to the resting position
   */
  public CommandBase restingPositionCommand() {
    return setArmPositionCommand(
        Constants.PinkArm.Extension.restingLength, Constants.PinkArm.Joint.restingAngle);
  }

  public CommandBase setExtensionLengthCommand(double length) {
    return Commands.runOnce(() -> setExtensionGoalLength(length), this);
  }

  /**
   * Manual Joint and Arm Control Command
   *
   * @param jointControl Input axis to change joint angle
   * @param armRetract Input button to retract the arm
   * @param armExtend Input button to extend the arm
   * @return Command for manually controlling the pink arm
   */
  public CommandBase armManualControlCommand(
      DoubleSupplier jointControl, BooleanSupplier armRetract, BooleanSupplier armExtend) {
    return Commands.run(
        () -> {
          setJointGoalAngle(
              getJointAngle()
                  + jointControl.getAsDouble() * Constants.PinkArm.Joint.incrementJointAngle);

          if (armRetract.getAsBoolean()) {
            setExtensionGoalLength(
                getExtensionLength() - Constants.PinkArm.Extension.incrementExtensionLength);
          } else if (armExtend.getAsBoolean()) {
            setExtensionGoalLength(
                getExtensionLength() + Constants.PinkArm.Extension.incrementExtensionLength);
          }
        },
        this);
  }

  /**
   * Get the absolute angle of the joint
   *
   * @return Joint angle in degrees
   */
  public double getJointAngle() {
    return (jointAbsolutePosSignal.getValue() * 360.0) + 90.0;
  }

  /** Trim the joint's absolute encoder. The current position will become the new zero */
  public void updateJointEncoderOffset() {
    double currentOffset = jointEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - jointAbsolutePosSignal.getValue()) % 1.0;
    Preferences.setDouble("PinkArmJointEncoderOffset", offset * 360.0);
    jointEncoderConfig.MagnetSensor.MagnetOffset = offset;
    jointEncoder.getConfigurator().apply(jointEncoderConfig);
  }

  public void pointToPosition(Translation3d fieldPos) {
    pointArmExtensionToPosition(fieldPos);
    pointArmJointToPosition(fieldPos);
  }

  public void pointArmExtensionToPosition(Translation3d fieldPos) {
    double aimPointForward =
        RobotContainer.isCubeMode()
            ? Constants.PointToPosition.cubeForward
            : Constants.PointToPosition.coneForward;

    Translation3d jointPos =
        new Translation3d(
            RobotContainer.swerve.getPose().getX(),
            RobotContainer.swerve.getPose().getY(),
            Constants.PointToPosition.jointHeight);
    double distanceToTarget = jointPos.getDistance(fieldPos);

    double goalExtension =
        distanceToTarget - Constants.PointToPosition.jointToExtensionLength - aimPointForward;
    if (goalExtension > Constants.PinkArm.Extension.FORWARD_LIMIT
        || Math.abs(getJointAngle() - 90) < 20) {
      goalExtension = 0.0;
    }
    setExtensionGoalLength(goalExtension);
  }

  public void pointArmJointToPosition(Translation3d fieldPos) {
    double aimPointForward =
        RobotContainer.isCubeMode()
            ? Constants.PointToPosition.cubeForward
            : Constants.PointToPosition.coneForward;
    double aimPointDown =
        RobotContainer.isCubeMode()
            ? Constants.PointToPosition.cubeDown
            : Constants.PointToPosition.coneDown;

    Pose2d robotPose = RobotContainer.swerve.getPose();
    Translation3d jointPos =
        new Translation3d(
            robotPose.getX(), robotPose.getY(), Constants.PointToPosition.jointHeight);

    double distanceToTarget = jointPos.getDistance(fieldPos);
    double angleToTarget =
        Math.asin((fieldPos.getZ() - Constants.PointToPosition.jointHeight) / distanceToTarget);

    double d = distanceToTarget + aimPointForward;
    double cosC = (Math.pow(d, 2) + Math.pow(d, 2) - Math.pow(aimPointDown, 2)) / (2 * d * d);
    double angleOffset = Math.copySign(Math.acos(cosC), aimPointDown);

    setJointGoalAngle(Units.radiansToDegrees(angleToTarget + angleOffset));
  }

  /**
   * sets the extension length and joint angle to the scoring placement goal
   *
   * @param targetPos a Translation3d in field coordinates
   * @return Command to point the pink arm to a position on the field at all times
   */
  public CommandBase pointToPositionCommand(Translation3d targetPos) {
    return Commands.run(() -> pointToPosition(targetPos), this);
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> jointMotor.setControl(new DutyCycleOut(-0.2)), this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (getJointVelocity() == 0) {
                    addFault(
                        "[System Check] Joint absolute velocity measured too slow", false, true);
                  }
                  jointMotor.setControl(new StaticBrake());
                },
                this),
            Commands.runOnce(() -> extensionMotor.setControl(new DutyCycleOut(0.3)), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (getExtensionVelocity() < 0.1) {
                    addFault("[System Check] Extension velocity measured too slow", false, true);
                  }
                  extensionMotor.setControl(new StaticBrake());
                },
                this),
            Commands.runOnce(() -> setJointGoalAngle(0), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getJointAngle() < -15 || getJointAngle() > 15) {
                    addFault("[System Check] Joint did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setJointGoalAngle(90), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getJointAngle() < 75 || getJointAngle() > 115) {
                    addFault("[System Check] Joint did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setExtensionGoalLength(0.5), this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (getExtensionLength() < 0.45 || getExtensionLength() > 0.55) {
                    addFault("[System Check] Extension did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setExtensionGoalLength(0), this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getExtensionLength()) > 0.05) {
                    addFault("[System Check] Extension did not reach target position", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(
            Commands.runOnce(
                () -> {
                  jointMotor.setControl(new StaticBrake());
                  extensionMotor.setControl(new StaticBrake());
                },
                this));
  }
}
