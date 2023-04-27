package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.sim.Pigeon2SimState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.subsystem.SubsystemFault;
import frc.lib.swerve.Mk4SwerveModulePro;
import frc.lib.util.Vector3;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotPoseLookup;

public class Swerve extends AdvancedSubsystem {
  protected final SwerveDrivePoseEstimator odometry;
  public final SwerveDriveKinematics kinematics;

  protected final Mk4SwerveModulePro[] modules;

  protected final Pigeon2 imu;
  protected final Pigeon2SimState imuSim;
  protected final StatusSignalValue<Double> imuRollSignal;
  protected final StatusSignalValue<Double> imuPitchSignal;
  protected final StatusSignalValue<Double> imuYawSignal;
  //  protected final StatusSignalValue<Double> imuAccelXSignal;
  //  protected final StatusSignalValue<Double> imuAccelYSignal;
  protected final StatusSignalValue<Double> imuAccelZSignal;

  private final RobotPoseLookup poseLookup;

  protected double teleopVelConstraint;
  protected double teleopAngularVelConstraint;

  protected final Field2d field2d = new Field2d();

  public Swerve() {
    poseLookup = new RobotPoseLookup();

    imu = new Pigeon2(Constants.Swerve.imuCanID, Constants.canivoreBusName);
    Pigeon2Configuration imuConfig = new Pigeon2Configuration();
    imu.getConfigurator().apply(imuConfig);
    zeroIMU();

    imuSim = imu.getSimState();

    teleopVelConstraint = Constants.Swerve.maxVelTele;
    teleopAngularVelConstraint = Constants.Swerve.maxAngularVelTele;

    imuRollSignal = imu.getRoll();
    imuPitchSignal = imu.getPitch();
    imuYawSignal = imu.getYaw();
    //    imuAccelXSignal = imu.getAccelerationX();
    //    imuAccelYSignal = imu.getAccelerationY();
    imuAccelZSignal = imu.getAccelerationZ();

    modules =
        new Mk4SwerveModulePro[] {
          new Mk4SwerveModulePro(
              Mk4SwerveModulePro.ModuleCode.FL,
              Constants.Swerve.FrontLeftModule.driveMotorCanID,
              Constants.Swerve.FrontLeftModule.rotationMotorCanID,
              Constants.Swerve.FrontLeftModule.rotationEncoderCanID,
              Constants.canivoreBusName), // FL
          new Mk4SwerveModulePro(
              Mk4SwerveModulePro.ModuleCode.FR,
              Constants.Swerve.FrontRightModule.driveMotorCanID,
              Constants.Swerve.FrontRightModule.rotationMotorCanID,
              Constants.Swerve.FrontRightModule.rotationEncoderCanID,
              Constants.canivoreBusName), // FR
          new Mk4SwerveModulePro(
              Mk4SwerveModulePro.ModuleCode.BL,
              Constants.Swerve.BackLeftModule.driveMotorCanID,
              Constants.Swerve.BackLeftModule.rotationMotorCanID,
              Constants.Swerve.BackLeftModule.rotationEncoderCanID,
              Constants.canivoreBusName), // BL
          new Mk4SwerveModulePro(
              Mk4SwerveModulePro.ModuleCode.BR,
              Constants.Swerve.BackRightModule.driveMotorCanID,
              Constants.Swerve.BackRightModule.rotationMotorCanID,
              Constants.Swerve.BackRightModule.rotationEncoderCanID,
              Constants.canivoreBusName) // BR
        };

    kinematics =
        new SwerveDriveKinematics(
            Constants.Swerve.FrontLeftModule.moduleOffset,
            Constants.Swerve.FrontRightModule.moduleOffset,
            Constants.Swerve.BackLeftModule.moduleOffset,
            Constants.Swerve.BackRightModule.moduleOffset);

    odometry =
        new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getPositions(),
            new Pose2d(),
            Constants.Swerve.Odometry.stateStdDevs,
            Constants.Swerve.Odometry.visionStdDevs);

    registerHardware("IMU", imu);

    SmartDashboard.putData("Field", field2d);
    SmartDashboard.putData("Trim Modules", zeroModulesCommand());
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    Pose2d currentPose = odometry.update(getYaw(), getPositions());
    double correctTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Swerve/OdomRuntime", correctTimeMS);
    //    field2d.setRobotPose(currentPose);
    poseLookup.addPose(currentPose);

    SmartDashboard.putNumberArray(
        "Swerve/Odometry",
        new double[] {
          currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()
        });

    //    SmartDashboard.putNumberArray(
    //        "Swerve/ModuleStates",
    //        new double[] {
    //          modules[0].getAbsoluteRotationDegrees(),
    // modules[0].getDriveVelocityMetersPerSecond(),
    //          modules[1].getAbsoluteRotationDegrees(),
    // modules[1].getDriveVelocityMetersPerSecond(),
    //          modules[2].getAbsoluteRotationDegrees(),
    // modules[2].getDriveVelocityMetersPerSecond(),
    //          modules[3].getAbsoluteRotationDegrees(),
    // modules[3].getDriveVelocityMetersPerSecond(),
    //        });

    //    SmartDashboard.putNumberArray(
    //        "Swerve/TargetModuleStates",
    //        new double[] {
    //          modules[0].getTargetState().angle.getDegrees(),
    //              modules[0].getTargetState().speedMetersPerSecond,
    //          modules[1].getTargetState().angle.getDegrees(),
    //              modules[1].getTargetState().speedMetersPerSecond,
    //          modules[2].getTargetState().angle.getDegrees(),
    //              modules[2].getTargetState().speedMetersPerSecond,
    //          modules[3].getTargetState().angle.getDegrees(),
    //              modules[3].getTargetState().speedMetersPerSecond,
    //        });

    //    Rotation3d orientation = getOrientation();
    //    SmartDashboard.putNumberArray(
    //        "Swerve/Orientation",
    //        new double[] {
    //          Units.radiansToDegrees(orientation.getX()),
    //          Units.radiansToDegrees(orientation.getY()),
    //          Units.radiansToDegrees(orientation.getZ())
    //        });

    //    Vector3 accel = getAcceleration();
    //    SmartDashboard.putNumberArray(
    //        "Swerve/Acceleration", new double[] {accel.getX(), accel.getY(), accel.getZ()});
    //    SmartDashboard.putNumber("Swerve/Pitch", getPitch());

    if (!DriverStation.isAutonomousEnabled()) {
      correctOdom(RobotContainer.driver.getBackButton(), RobotContainer.driver.getBackButton());
    }

    StatusSignalValue.waitForAll(0, imuRollSignal, imuPitchSignal, imuYawSignal, imuAccelZSignal);
    //    imuRollSignal.refresh();
    //    imuPitchSignal.refresh();
    //    imuYawSignal.refresh();
    //    imuAccelXSignal.refresh();
    //    imuAccelYSignal.refresh();
    //    imuAccelZSignal.refresh();

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    SmartDashboard.putNumber("Swerve/PeriodicRuntime", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(getStates());
    double currentYaw = getYaw().getDegrees();

    imuSim.setRawYaw(
        currentYaw + (Units.radiansToDegrees(currentSpeeds.omegaRadiansPerSecond) * 0.02));
  }

  public Vector3 getGravityVector() {
    return new Vector3(
        imu.getGravityVectorX().getValue(),
        imu.getGravityVectorY().getValue(),
        imu.getGravityVectorZ().getValue());
  }

  /**
   * Drive the robot with field relative chassis speeds
   *
   * @param fieldRelativeSpeeds Field relative chassis speeds
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond,
            fieldRelativeSpeeds.omegaRadiansPerSecond,
            getPose().getRotation());

    driveRobotRelative(speeds);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Mk4SwerveModulePro.DRIVE_MAX_VEL);

    setModuleStates(targetStates);
  }

  /** Zeros the IMU yaw */
  public void zeroIMU() {
    imu.setYaw(0);
  }

  /**
   * Get the yaw from the IMU
   *
   * @return Rotation2d representing the yaw of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(imuYawSignal.getValue());
  }

  public double getPitch() {
    return imuPitchSignal.getValue();
  }

  public double getRoll() {
    return imuRollSignal.getValue();
  }

  /**
   * Get the 3d orientation of the robot
   *
   * @return Rotation3d representing the robot orientation
   */
  public Rotation3d getOrientation() {
    return new Rotation3d(
        Units.degreesToRadians(getRoll()),
        Units.degreesToRadians(getPitch()),
        getYaw().getRadians());
  }

  /**
   * Get the current acceleration of the robot from the IMU
   *
   * @return Vector3 representing the current acceleration
   */
  public Vector3 getAcceleration() {
    return new Vector3(
        imu.getAccelerationX().getValue(),
        imu.getAccelerationY().getValue(),
        imu.getAccelerationZ().getValue());
  }

  /**
   * Get the current state of all the swerve modules
   *
   * @return Array of current swerve module states
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Get the current positions of all the swerve modules
   *
   * @return Array of current swerve module positions
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPositions();
    }
    return positions;
  }

  /**
   * Reset the odometry
   *
   * @param pose The pose to reset the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }

  /**
   * Get the estimated robot pose from the odometry
   *
   * @return Pose2d representing estimated robot pose
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Set the desired states of all the swerve modules
   *
   * @param states Array of desired states, in the same order as the modules array
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  /**
   * Set the teleop driving constraints
   *
   * @param maxVel Max robot velocity in m/s
   * @param maxAngularVel Max robot angular velocity in rad/s
   */
  public void setTeleopConstraints(double maxVel, double maxAngularVel) {
    teleopVelConstraint = maxVel;
    teleopAngularVelConstraint = maxAngularVel;
  }

  /**
   * Get the teleop velocity constraint
   *
   * @return Max teleop velocity in m/s
   */
  public double getTeleopVelConstraint() {
    return teleopVelConstraint;
  }

  /**
   * Get the teleop angular velocity constraint
   *
   * @return Max teleop velocity in rad/s
   */
  public double getTeleopAngularVelConstraint() {
    return teleopAngularVelConstraint;
  }

  public void lockModules() {
    for (Mk4SwerveModulePro module : modules) {
      module.lockModule();
    }
  }

  /**
   * Command used to trim all the modules' absolute encoders
   *
   * @return Command to trim the modules, runs while disabled
   */
  public CommandBase zeroModulesCommand() {
    return Commands.runOnce(
            () -> {
              for (Mk4SwerveModulePro module : modules) {
                module.updateRotationOffset();
              }
            })
        .ignoringDisable(true);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public CommandBase autoBalance(double maxVel) {
    PIDController controller =
        new PIDController(
            Constants.AutoBalance.BALANCE_CONSTANTS.kP,
            Constants.AutoBalance.BALANCE_CONSTANTS.kI,
            Constants.AutoBalance.BALANCE_CONSTANTS.kD);
    controller.setTolerance(3);

    Timer lockTimer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              controller.reset();
              lockTimer.stop();
              lockTimer.reset();
            },
            this),
        Commands.run(
                () -> {
                  Pose3d robotOrientation =
                      new Pose3d(
                          new Translation3d(),
                          new Rotation3d(
                              Units.degreesToRadians(getRoll()),
                              Units.degreesToRadians(getPitch()),
                              getPose().getRotation().getRadians()));

                  Pose3d chargeStationOrientation =
                      robotOrientation.transformBy(
                          new Transform3d(
                              new Translation3d(),
                              new Rotation3d(0, 0, -getPose().getRotation().getRadians())));

                  double chargeStationPitch =
                      Units.radiansToDegrees(chargeStationOrientation.getRotation().getY());

                  double rollVel = imu.getAngularVelocityX().getValue();
                  double pitchVel = imu.getAngularVelocityY().getValue();
                  double angularVel = Math.sqrt(Math.pow(rollVel, 2) + Math.pow(pitchVel, 2));

                  if (Math.abs(chargeStationPitch) < 3 || angularVel > 10) {
                    lockModules();
                    lockTimer.start();
                  } else {
                    lockTimer.stop();
                    lockTimer.reset();
                    double x = controller.calculate(chargeStationPitch, 0);
                    if (Math.abs(x) > maxVel) {
                      x = Math.copySign(maxVel, x);
                    }
                    driveFieldRelative(new ChassisSpeeds(x, 0, 0));
                  }
                },
                this)
            .until(() -> lockTimer.hasElapsed(0.25)),
        Commands.run(this::lockModules, this));
  }

  public void correctOdom(boolean force, boolean correctYaw) {
    double time = Timer.getFPGATimestamp();

    Pose2d leftBotPose = null;
    double leftPoseTimestamp =
        time
            - ((LimelightHelpers.getLatency_Capture("limelight-left")
                    + LimelightHelpers.getLatency_Pipeline("limelight-left"))
                / 1000.0);
    Pose2d rightBotPose = null;
    double rightPoseTimestamp =
        time
            - ((LimelightHelpers.getLatency_Capture("limelight-right")
                    + LimelightHelpers.getLatency_Pipeline("limelight-right"))
                / 1000.0);

    Pose2d robotAtLeftCapture = poseLookup.lookup(leftPoseTimestamp);
    Pose2d robotAtRightCapture = poseLookup.lookup(rightPoseTimestamp);

    if (LimelightHelpers.getTV("limelight-left")) {
      Pose2d botpose =
          (DriverStation.getAlliance() == DriverStation.Alliance.Blue
              ? LimelightHelpers.getBotPose2d_wpiBlue("limelight-left")
              : LimelightHelpers.getBotPose2d_wpiRed("limelight-left"));

      if (botpose.getX() > 0.1
          && botpose.getX() < Constants.fieldSize.getX() - 0.1
          && botpose.getY() > 0.1
          && botpose.getY() < Constants.fieldSize.getY() - 1) {
        leftBotPose = botpose;
      }
    }

    if (LimelightHelpers.getTV("limelight-right")) {
      Pose2d botpose =
          (DriverStation.getAlliance() == DriverStation.Alliance.Blue
              ? LimelightHelpers.getBotPose2d_wpiBlue("limelight-right")
              : LimelightHelpers.getBotPose2d_wpiRed("limelight-right"));

      if (botpose.getX() > 0.1
          && botpose.getX() < Constants.fieldSize.getX() - 0.1
          && botpose.getY() > 0.1
          && botpose.getY() < Constants.fieldSize.getY() - 1) {
        rightBotPose = botpose;
      }
    }

    Pose2d correctionPose = null;
    Pose2d robotAtCorrectionPose = null;
    double correctionTimestamp = 0;
    Matrix<N3, N1> correctionDevs = null;

    if (leftBotPose != null && rightBotPose != null) {
      // Left and right have poses
      Pose2d leftToRightDiff = leftBotPose.relativeTo(rightBotPose);
      if (leftToRightDiff.getTranslation().getNorm() < 0.3
          && (!correctYaw || Math.abs(leftToRightDiff.getRotation().getDegrees()) < 15)) {
        // They agree
        correctionPose = leftBotPose.interpolate(rightBotPose, 0.5);
        robotAtCorrectionPose = robotAtLeftCapture.interpolate(robotAtRightCapture, 0.5);
        correctionTimestamp = (leftPoseTimestamp + rightPoseTimestamp) / 2.0;
        correctionDevs = Constants.Swerve.Odometry.visionStdDevsTrust;
      } else {
        // They don't agree
        Pose2d leftDiff = leftBotPose.relativeTo(robotAtLeftCapture);
        Pose2d rightDiff = rightBotPose.relativeTo(robotAtRightCapture);
        double leftDist = leftDiff.getTranslation().getNorm();
        double rightDist = rightDiff.getTranslation().getNorm();

        if ((leftDist < 2.0 || force) && leftDist <= rightDist) {
          // Left closest
          if (!correctYaw || force || Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
            correctionPose = leftBotPose;
            robotAtCorrectionPose = robotAtLeftCapture;
            correctionTimestamp = leftPoseTimestamp;
            correctionDevs = Constants.Swerve.Odometry.visionStdDevs;
          }
        } else if ((rightDist < 2.0 || force) && rightDist <= leftDist) {
          // Right closest
          if (!correctYaw || force || Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
            correctionPose = rightBotPose;
            robotAtCorrectionPose = robotAtRightCapture;
            correctionTimestamp = rightPoseTimestamp;
            correctionDevs = Constants.Swerve.Odometry.visionStdDevs;
          }
        }
      }
    } else if (leftBotPose != null) {
      Pose2d leftDiff = leftBotPose.relativeTo(robotAtLeftCapture);
      double leftDist = leftDiff.getTranslation().getNorm();

      if (leftDist < 2.0 || force) {
        if (!correctYaw || force || Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
          correctionPose = leftBotPose;
          robotAtCorrectionPose = robotAtLeftCapture;
          correctionTimestamp = leftPoseTimestamp;
          correctionDevs = Constants.Swerve.Odometry.visionStdDevs;
        }
      }
    } else if (rightBotPose != null) {
      Pose2d rightDiff = rightBotPose.relativeTo(robotAtRightCapture);
      double rightDist = rightDiff.getTranslation().getNorm();

      if (rightDist < 2.0 || force) {
        if (!correctYaw || force || Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
          correctionPose = rightBotPose;
          robotAtCorrectionPose = robotAtRightCapture;
          correctionTimestamp = rightPoseTimestamp;
          correctionDevs = Constants.Swerve.Odometry.visionStdDevs;
        }
      }
    }

    if (correctionPose != null) {
      odometry.addVisionMeasurement(
          (correctYaw)
              ? correctionPose
              : new Pose2d(correctionPose.getTranslation(), robotAtCorrectionPose.getRotation()),
          correctionTimestamp,
          correctionDevs);
    }

    if (leftBotPose != null) {
      SmartDashboard.putNumberArray(
          "Swerve/LLPoseLeft",
          new double[] {
            leftBotPose.getX(), leftBotPose.getY(), leftBotPose.getRotation().getDegrees()
          });
    }
    if (rightBotPose != null) {
      SmartDashboard.putNumberArray(
          "Swerve/LLPoseRight",
          new double[] {
            rightBotPose.getX(), rightBotPose.getY(), rightBotPose.getRotation().getDegrees()
          });
    }
  }

  @Override
  protected CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  modules[0].getSystemCheckCommand().schedule();
                  modules[1].getSystemCheckCommand().schedule();
                  modules[2].getSystemCheckCommand().schedule();
                  modules[3].getSystemCheckCommand().schedule();
                },
                this),
            // Hack to run module system checks since modules[] does not exist when this method is
            // called
            Commands.waitUntil(
                () ->
                    modules[0].getCurrentCommand() == null
                        && modules[1].getCurrentCommand() == null
                        && modules[2].getCurrentCommand() == null
                        && modules[3].getCurrentCommand() == null),
            Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds(0, 0, 0.5))),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  driveFieldRelative(new ChassisSpeeds());
                  if (-imu.getRate() < Units.radiansToDegrees(0.3)) {
                    addFault("[System Check] IMU rate too low", false, true);
                  }
                },
                this),
            Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds(0, 0, -0.5))),
            Commands.waitSeconds(2.0),
            Commands.runOnce(
                () -> {
                  driveFieldRelative(new ChassisSpeeds());
                  if (-imu.getRate() > Units.radiansToDegrees(-0.3)) {
                    addFault("[System Check] IMU rate too low", false, true);
                  }
                },
                this))
        .until(
            () ->
                getFaults().size() > 0
                    || modules[0].getFaults().size() > 0
                    || modules[1].getFaults().size() > 0
                    || modules[2].getFaults().size() > 0
                    || modules[3].getFaults().size() > 0)
        .andThen(Commands.runOnce(() -> driveFieldRelative(new ChassisSpeeds()), this));
  }

  @Override
  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : this.getFaults()) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }

    for (Mk4SwerveModulePro module : modules) {
      SystemStatus moduleStatus = module.getSystemStatus();
      if (moduleStatus == SystemStatus.ERROR) {
        worstStatus = SystemStatus.ERROR;
      } else if (moduleStatus == SystemStatus.WARNING && worstStatus == SystemStatus.OK) {
        worstStatus = SystemStatus.WARNING;
      }
    }

    return worstStatus;
  }
}
