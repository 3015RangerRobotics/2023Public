package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ScoringTracker;

public class Jaw extends AdvancedSubsystem {
  public enum State {
    FLOOR_PICKUP,
    FLOOR_TIPPED_PICKUP,
    FLOOR_BEHIND_PICKUP,
    SHELF_PICKUP,
    CHUTE_PICKUP,
    SCORING_POS,
    SCORING_POS_LOW,
    IDLE
  }

  protected final CANSparkMax jawMotor;
  protected final SparkMaxPIDController jawController;
  protected final RelativeEncoder jawEncoder;
  protected final AbsoluteEncoder jawAbsoluteEncoder;

  protected State state;

  public Jaw() {
    jawMotor =
        new CANSparkMax(Constants.Jaw.pivotMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    jawMotor.restoreFactoryDefaults();
    jawMotor.setInverted(true);
    jawMotor.setSmartCurrentLimit(Constants.Jaw.pivotCurrentLimit);
    jawMotor.enableVoltageCompensation(12);
    jawMotor.setClosedLoopRampRate(0.05);

    jawAbsoluteEncoder = jawMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    jawAbsoluteEncoder.setPositionConversionFactor(360.0);
    jawAbsoluteEncoder.setVelocityConversionFactor(360.0);
    jawAbsoluteEncoder.setInverted(false);
    jawAbsoluteEncoder.setZeroOffset(Preferences.getDouble("JawEncoderOffset", 0.0));

    jawEncoder = jawMotor.getEncoder();
    jawEncoder.setPositionConversionFactor(Constants.Jaw.pivotDegreesPerPulse);
    jawEncoder.setVelocityConversionFactor(Constants.Jaw.pivotDegreesPerPulse);

    jawEncoder.setPosition(jawAbsoluteEncoder.getPosition());

    jawController = jawMotor.getPIDController();
    jawController.setP(Constants.Jaw.pivotP);
    jawController.setI(Constants.Jaw.pivotI);
    jawController.setD(Constants.Jaw.pivotD);
    jawController.setFeedbackDevice(jawAbsoluteEncoder);
    jawController.setPositionPIDWrappingEnabled(true);
    jawController.setPositionPIDWrappingMinInput(0.0);
    jawController.setPositionPIDWrappingMaxInput(360.0);

    state = State.SCORING_POS;

    registerHardware("Jaw Motor", jawMotor);

    SmartDashboard.putData("Jaw/TrimJawEncoder", trimJawEncoderCommand());
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    boolean cubeMode = RobotContainer.isCubeMode();

    switch (state) {
      case SCORING_POS:
        if (cubeMode) {
          setDesiredPivotAngle(Constants.Jaw.pivotCubeModePos);
        } else {
          ScoringTracker.getScoringTrackerOverride()
              .ifPresentOrElse(
                  (Integer id) -> {
                    if (id >= 18 && !DriverStation.isAutonomous()) {
                      setDesiredPivotAngle(Constants.Jaw.pivotConeScoringPosLow);
                    } else {
                      setDesiredPivotAngle(Constants.Jaw.pivotConeScoringPos);
                    }
                  },
                  () -> setDesiredPivotAngle(Constants.Jaw.pivotConeScoringPos));
        }
        break;
      case CHUTE_PICKUP:
      case FLOOR_TIPPED_PICKUP:
        if (cubeMode) {
          setDesiredPivotAngle(Constants.Jaw.pivotCubeModePos);
        } else {
          setDesiredPivotAngle(Constants.Jaw.pivotConePickupTippedPos);
        }
        break;
      case SHELF_PICKUP:
        if (cubeMode) {
          setDesiredPivotAngle(Constants.Jaw.pivotCubeModePos);
        } else {
          setDesiredPivotAngle(Constants.Jaw.pivotConeShelfPos);
        }
        break;
      case FLOOR_PICKUP:
        if (cubeMode) {
          setDesiredPivotAngle(Constants.Jaw.pivotCubeModePos);
        } else {
          setDesiredPivotAngle(Constants.Jaw.pivotConePickupStandingPos);
        }
        break;
      case FLOOR_BEHIND_PICKUP:
        if (cubeMode) {
          setDesiredPivotAngle(Constants.Jaw.pivotCubeModePos);
        } else {
          setDesiredPivotAngle(Constants.Jaw.pivotConePickupBehindPos);
        }
        break;
      case IDLE:
        // Do nothing
        break;
    }

    SmartDashboard.putNumber("Jaw/JawTemp", jawMotor.getMotorTemperature());

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Jaw/PeriodicRuntime", runtimeMS);
  }

  /**
   * Get the current pivot angle
   *
   * @return Pivot angle in degrees
   */
  public double getPivotAngle() {
    return jawAbsoluteEncoder.getPosition();
  }

  /**
   * Set the desired angle of the pivot
   *
   * @param angle Pivot angle in degrees
   */
  public void setDesiredPivotAngle(double angle) {
    jawController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setState(State state) {
    this.state = state;
  }

  public CommandBase setJawStateCommand(State state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  public CommandBase trimJawEncoderCommand() {
    return Commands.runOnce(
            () -> {
              double currentOffset = jawAbsoluteEncoder.getZeroOffset();
              double newOffset = currentOffset + jawAbsoluteEncoder.getPosition();

              if (newOffset > 360) {
                newOffset -= 360;
              }

              Preferences.setDouble("JawEncoderOffset", newOffset);
              jawAbsoluteEncoder.setZeroOffset(newOffset);
              jawEncoder.setPosition(0.0);
            },
            this)
        .ignoringDisable(true);
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            setJawStateCommand(State.IDLE),
            Commands.runOnce(() -> jawMotor.set(0.5), this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(jawEncoder.getVelocity()) < 90) {
                    addFault("[System Check] Jaw motor encoder measured too slow", false, true);
                  }
                  if (Math.abs(jawAbsoluteEncoder.getVelocity()) < 90) {
                    addFault("[System Check] Jaw absolute encoder measured too slow", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> setDesiredPivotAngle(100), this),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  if (getPivotAngle() > 150 || getPivotAngle() < 50) {
                    addFault("[System Check] Jaw did not reach target position", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> jawMotor.set(0), this));
  }
}
