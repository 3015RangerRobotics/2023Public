package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDStrip extends AdvancedSubsystem {
  public enum State {
    PARTY,
    NORMAL,
    READY_TO_AIM,
    SAD
  }

  protected final CANdle candle;
  private State state = State.NORMAL;

  public LEDStrip() {
    candle = new CANdle(Constants.LEDStrip.candleID, Constants.canivoreBusName);

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = CANdle.LEDStripType.GRBW;
    config.v5Enabled = true;
    config.disableWhenLOS = true;
    config.statusLedOffWhenActive = true;
    config.vBatOutputMode = CANdle.VBatOutputMode.Off;

    candle.configAllSettings(config);
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    if (DriverStation.isDisabled()) {
      SystemStatus swerveStatus = RobotContainer.swerve.getSystemStatus();
      for (var module : RobotContainer.swerve.modules) {
        SystemStatus moduleStatus = module.getSystemStatus();
        if (moduleStatus == SystemStatus.ERROR) {
          swerveStatus = SystemStatus.ERROR;
        } else if (moduleStatus == SystemStatus.WARNING && swerveStatus == SystemStatus.OK) {
          swerveStatus = SystemStatus.WARNING;
        }
      }

      setStatusLED(swerveStatus, Constants.LEDStrip.swerveLED);
      setStatusLED(RobotContainer.pinkArm.getSystemStatus(), Constants.LEDStrip.pinkArmLED);
      setStatusLED(RobotContainer.turret.getSystemStatus(), Constants.LEDStrip.turretLED);
      setStatusLED(RobotContainer.intake.getSystemStatus(), Constants.LEDStrip.intakeLED);
      setStatusLED(RobotContainer.jaw.getSystemStatus(), Constants.LEDStrip.jawLED);
      //      setStatusLED(RobotContainer.stealer.getSystemStatus(), Constants.LEDStrip.stealerLED);
      setStatusLED(RobotContainer.manhattan.getSystemStatus(), Constants.LEDStrip.manhattanLED);
    }

    switch (state) {
      case NORMAL:
        if (DriverStation.isDisabled()) {
          if (DriverStation.isDSAttached()) {
            if (SmartDashboard.getString("Autonomous Mode/selected", "None").equals("None")) {
              sadAnimation();
            } else {
              allianceAnim();
            }
          } else {
            breathingWhite();
          }
        } else if (DriverStation.isAutonomousEnabled()) {
          fireAnimation();
        } else {
          if (RobotContainer.isCubeMode()) {
            cubeModeAnimation();
          } else {
            coneModeAnimation();
          }
        }
        break;
      case PARTY:
        partyAnim();
        break;
      case SAD:
        sadAnimation();
        break;
      case READY_TO_AIM:
        readyToAimAnim();
        break;
    }

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("LEDStrip/PeriodicRuntime", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {}

  public void setState(State state) {
    this.state = state;
  }

  protected void setStatusLED(SystemStatus status, int led) {
    switch (status) {
      case OK:
        // Led Green
        candle.setLEDs(0, 50, 0, 0, led, 1);
        break;
      case WARNING:
        // Led Yellow
        candle.setLEDs(50, 35, 0, 0, led, 1);
        break;
      case ERROR:
        // Led Red
        candle.setLEDs(50, 0, 0, 0, led, 1);
        break;
    }
  }

  public void ledStripOff() {
    candle.clearAnimation(0);
  }

  public void readyToAimAnim() {
    candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.2, Constants.LEDStrip.numLEDs, 8));
  }

  public void partyAnim() {
    candle.animate(new RainbowAnimation(1.0, 1.0, Constants.LEDStrip.numLEDs, false, 8));
  }

  public void sadAnimation() {
    candle.animate(new StrobeAnimation(255, 0, 0, 0, 1.0, Constants.LEDStrip.numLEDs, 8));
  }

  public void breathingWhite() {
    candle.animate(new SingleFadeAnimation(0, 0, 0, 255, 0.5, Constants.LEDStrip.numLEDs, 8));
  }

  private void allianceAnim() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      candle.animate(
          new LarsonAnimation(
              255,
              0,
              0,
              0,
              0.03,
              Constants.LEDStrip.numLEDs,
              LarsonAnimation.BounceMode.Center,
              2,
              8));
    } else {
      candle.animate(
          new LarsonAnimation(
              0,
              0,
              255,
              0,
              0.03,
              Constants.LEDStrip.numLEDs,
              LarsonAnimation.BounceMode.Center,
              2,
              8));
    }
  }

  private void fireAnimation() {
    candle.animate(new FireAnimation(0.8, 1.0, Constants.LEDStrip.numLEDs * 3, 1.0, 1.0, false, 8));
  }

  public void coneModeAnimation() {
    candle.animate(new StrobeAnimation(255, 180, 0, 0, 1.0, Constants.LEDStrip.numLEDs, 8));
  }

  public void cubeModeAnimation() {
    candle.animate(new StrobeAnimation(180, 0, 255, 0, 1.0, Constants.LEDStrip.numLEDs, 8));
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.none();
  }
}
