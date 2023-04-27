package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleOn;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.LEDStrip;
import frc.robot.util.ScoringTracker;
import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command autoCommand;

  @Override
  public void robotInit() {
    instance = this;

    DriverStation.silenceJoystickConnectionWarning(true);

    if (RobotBase.isReal()) {
      IntakeCam.init();
      IntakeCam.getConeOffsetMeters();
    }

    // Start data logging
    //    if (RobotBase.isReal()) {
    //      DataLogManager.start();
    //      DriverStation.startDataLog(DataLogManager.getLog());
    //    }

    // Set the match name to practice initially
    SmartDashboard.putString("matchName", "practice");

    // Initialize all robot systems
    RobotContainer.init();

    // Initialize auto modes
    Autos.init();

    // Start the pathplanner server. Comment this out if it is not needed.
    //    PathPlannerServer.startServer(5812);

    //    addPeriodic(BatteryUsage::publishUsage, 1.0);
    addPeriodic(() -> SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime()), 1.0);

    //    PPSwerveControllerCommand.setLoggingCallbacks(
    //        activeTrajectory -> {
    //          List<Double> statePoses = new ArrayList<>();
    //          for (Trajectory.State s : activeTrajectory.getStates()) {
    //            PathPlannerTrajectory.PathPlannerState state =
    //                (PathPlannerTrajectory.PathPlannerState) s;
    //            statePoses.addAll(
    //                List.of(
    //                    state.poseMeters.getX(),
    //                    state.poseMeters.getY(),
    //                    state.holonomicRotation.getDegrees()));
    //          }
    //          SmartDashboard.putNumberArray(
    //              "PathFollowing/ActiveTrajectory", statePoses.toArray(new Double[] {}));
    //        },
    //        targetPose ->
    //            SmartDashboard.putNumberArray(
    //                "PathFollowing/TargetPose",
    //                new double[] {
    //                  targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()
    //                }),
    //        setpoint -> {
    //          SmartDashboard.putNumber("PathFollowing/TargetXVel", setpoint.vxMetersPerSecond);
    //          SmartDashboard.putNumber("PathFollowing/TargetYVel", setpoint.vyMetersPerSecond);
    //          SmartDashboard.putNumber(
    //              "PathFollowing/TargetAngularVel",
    //              Units.radiansToDegrees(setpoint.omegaRadiansPerSecond));
    //        },
    //        (translationError, rotationError) -> {
    //          SmartDashboard.putNumber("PathFollowing/XPosError", translationError.getX());
    //          SmartDashboard.putNumber("PathFollowing/YPosError", translationError.getY());
    //          SmartDashboard.putNumber("PathFollowing/RotationError", rotationError.getDegrees());
    //        });
  }

  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();

    LimelightHelpers.setPipelineIndex("limelight-center", 1);

    // Hack to get the stupid JSON parsing freeze out of the way (dear WPILib please use a different
    // JSON library)
    if (Timer.getFPGATimestamp() > 5.0 && Timer.getFPGATimestamp() < 30.0) {
      LimelightHelpers.getLatestResults("limelight-rear");
      LimelightHelpers.getLatestResults("limelight-center");
    }

    SmartDashboard.putNumber("RIOInputVoltage", RobotController.getInputVoltage());
    SmartDashboard.putNumber(
        "RIOCANUtil", RobotController.getCANStatus().percentBusUtilization * 100);

    //    logComponentPoses();
    //    logScoredPositions();

    //    if (RobotBase.isReal()) {
    //      System.out.println(IntakeCam.getConeOffsetMeters());
    //    }
    //    IntakeCam.doCalibrationFrame();

    SmartDashboard.putNumber("RobotPeriodicMS", (Timer.getFPGATimestamp() - startTime) * 1000);
  }

  @Override
  public void disabledInit() {
    RobotContainer.driver.setRumbleAnimation(new RumbleOff());
    RobotContainer.leds.setState(LEDStrip.State.NORMAL);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autoCommand = Autos.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime != -1 && matchTime <= 1) {
      RobotContainer.driver.setRumbleAnimation(new RumbleOn());
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public static void addPeriodicCallback(Runnable callback, double periodSeconds) {
    // Don't add the callback if the instance is null. This is pretty much just to make unit tests
    // work
    if (instance == null) {
      return;
    }

    instance.addPeriodic(callback, periodSeconds);
  }

  private void logScoredPositions() {
    boolean[] scoring = ScoringTracker.getScoring();

    List<Double> scoringPoses = new ArrayList<>();
    for (int i = 0; i < scoring.length; i++) {
      if (scoring[i]) {
        Translation3d scorePose = ScoringTracker.getScoringPos(i);
        Quaternion scoreQuat = new Rotation3d(0, Units.degreesToRadians(-90), 0).getQuaternion();
        scoringPoses.addAll(
            List.of(
                scorePose.getX(),
                scorePose.getY(),
                scorePose.getZ(),
                scoreQuat.getX(),
                scoreQuat.getY(),
                scoreQuat.getZ(),
                scoreQuat.getW()));
      }
    }

    SmartDashboard.putNumberArray("ScoredPositions", scoringPoses.toArray(new Double[] {}));
  }

  private void logComponentPoses() {
    List<Pose3d> componentPoses = new ArrayList<>();

    double turretAngle = Units.degreesToRadians(RobotContainer.turret.getTurretAngle());
    double pinkArmJointAngle = Units.degreesToRadians(-RobotContainer.pinkArm.getJointAngle());
    double pinkArmExtensionLength = RobotContainer.pinkArm.getExtensionLength();

    componentPoses.add(new Pose3d(0, 0, 0, new Rotation3d(0, 0, turretAngle))); // Turret
    componentPoses.add(
        new Pose3d(0, 0, 0.6, new Rotation3d(0, pinkArmJointAngle, turretAngle))
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, -0.6), new Rotation3d()))); // PinkArm Proximal
    componentPoses.add(
        new Pose3d(0, 0, 0.6, new Rotation3d(0, pinkArmJointAngle, turretAngle))
            .transformBy(
                new Transform3d(
                    new Translation3d(pinkArmExtensionLength / 2.0, 0, -0.6),
                    new Rotation3d()))); // PinkArm mid
    componentPoses.add(
        new Pose3d(0, 0, 0.6, new Rotation3d(0, pinkArmJointAngle, turretAngle))
            .transformBy(
                new Transform3d(
                    new Translation3d(pinkArmExtensionLength, 0, -0.6),
                    new Rotation3d()))); // PinkArm distal
    componentPoses.add(
        new Pose3d(0, 0, 0.6, new Rotation3d(0, pinkArmJointAngle, turretAngle))
            .transformBy(
                new Transform3d(
                    new Translation3d(pinkArmExtensionLength, 0, -0.6),
                    new Rotation3d()))); // Intake

    List<Double> logComponentPoses = new ArrayList<>();
    for (Pose3d pose : componentPoses) {
      logComponentPoses.addAll(
          List.of(
              pose.getX(),
              pose.getY(),
              pose.getZ(),
              pose.getRotation().getQuaternion().getW(),
              pose.getRotation().getQuaternion().getX(),
              pose.getRotation().getQuaternion().getY(),
              pose.getRotation().getQuaternion().getZ()));
    }
    SmartDashboard.putNumberArray("RobotComponentPoses", logComponentPoses.toArray(new Double[0]));
  }
}
