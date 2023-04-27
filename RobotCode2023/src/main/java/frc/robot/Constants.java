package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.FieldCalibration;
import frc.robot.util.Util;
import java.io.IOException;

public final class Constants {
  public static final String canivoreBusName = "*";

  public static final AprilTagFieldLayout apriltagLayout;

  static {
    try {
      apriltagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.02);

  public static final class CameraInfo {
    public static final double armLLToCone = Units.inchesToMeters(11.0);
    public static final double armJointToLL = Units.inchesToMeters(16.0);
  }

  public static final class PointToPosition {
    public static final double scoringTrackerCubeZOffset = 0.45;
    public static final double scoringTrackerCubeXOffset = Units.inchesToMeters(-9.0);
    public static final double scoringTrackerConeZOffset = 0.0;
    public static final double scoringTrackerConeXOffset = 0.0;
    public static final double jointToExtensionLength = Units.inchesToMeters(20.25);
    public static final double jointHeight = 0.55;

    public static final double cubeForward = 0.9;
    public static final double cubeDown = 0.0;

    public static final double coneForward = 0.18;
    public static final double coneDown = 0.35;
  }

  public static final class SafePositions {
    public static final double safeTurretAngle = 0.0;
    public static final double safeArmExtension = 0.0;
    public static final double safeJointAngleCommunity = 30.0;
    public static final double safeFieldPosition = 5.0;
    public static final double safeJointAngle = 80;
  }

  public enum IntakeMode {
    SHELF,
    CHUTE,
    FLOOR
  }

  public static final class PinkArm {
    public static final class Extension {
      public static final int motorID = 12;

      public static final double maxVel = 2.5;
      public static final double maxAccel = 6.0;
      public static final double maxJerk = 30.0;

      public static final double metersPerRotation = Units.inchesToMeters(1.0);

      public static final double kP = 5.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 5.0747 * metersPerRotation;
      public static final double kS = 0.64861;

      public static final double restingLength = 0.0;

      public static final double FORWARD_LIMIT = 1.0;
      public static final double REVERSE_LIMIT = 0.0;

      public static final double incrementExtensionLength = 0.15;

      public static final double cubeFloorIntakeLength = 0.06;
      public static final double coneFloorIntakeLength = 0.0;
      public static final double cubeFloorIntakeBehindLength = 0.20;
      public static final double coneFloorIntakeBehindLength = 0.12;
      public static final double coneTippedFloorIntakeLength = 0.0;

      public static final double extensionLengthChute = 0.0;
      public static final double extensionLengthShelf = 0.28;

      public static final double allowableExtensionError = 0.005;

      public static final class SimInfo {
        public static final double kV = 5.0747;
        public static final double kA = 0.12939;
      }
    }

    public static final class Joint {
      public static final int motorID = 15;
      public static final int encoderID = 11;

      public static final double gearing = 1.0 / 156.0;
      public static final double degreesPerRotation = RobotBase.isReal() ? 360.0 : gearing * 360.0;

      public static final double maxVel = 450; // 180.0; // deg/s
      public static final double maxAccel = 270.0; // 360.0; // deg/s^2
      public static final double maxJerk = 3600.0;

      public static final double kP = 150.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double restingAngle = -35.0;

      public static final double FORWARD_LIMIT = 140.0;
      public static final double REVERSE_LIMIT = -140.0;

      public static final double incrementJointAngle = 5.0;

      public static final double cubeFloorIntakeAngle = -29.0;
      public static final double coneFloorIntakeAngle = -6.0;
      public static final double coneTippedFloorIntakeAngle = -26.0;

      public static final double cubeFloorIntakeBehindAngle = 209.0;
      public static final double coneFloorIntakeBehindAngle = 210.0;

      public static final double jointAngleChute = 20.0;
      public static final double jointAngleShelf = 55;

      public static final class SimInfo {
        public static final double extendedJointToEndLength = Units.inchesToMeters(61.875);
        public static final double jointToEndMass = 5.0; // kg
        public static final double MOI =
            SingleJointedArmSim.estimateMOI(extendedJointToEndLength, jointToEndMass);
      }
    }
  }

  public static final class ScoreHighManual {
    public static final double conePinkArmLength = 0.87;
    public static final double conePinkArmAngle = 35;
    public static final double cubePinkArmLength = 0.55;
    public static final double cubePinkArmAngle = 25;
  }

  public static final class ScoreMidManual {
    public static final double conePinkArmLength = 0.3;
    public static final double conePinkArmAngle = 35;
    public static final double cubePinkArmLength = 0;
    public static final double cubePinkArmAngle = 15;
  }

  public static final class ScoreLowManual {
    public static final double conePinkArmLength = 0.0;
    public static final double conePinkArmAngle = 0.0;
    public static final double cubePinkArmLength = 0;
    public static final double cubePinkArmAngle = -10.0;
  }

  public static final class Turret {
    public static final int motorID = 34;

    public static final int encoderID = 10;

    public static final double degreesPerRotation = 360.0;

    public static final double maxVel = 70.0;
    public static final double maxAccel = 180.0;
    public static final double maxJerk = 3600.0;

    public static final double kP = 80.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.09402 * degreesPerRotation;
    public static final double kS = 0.57311;

    public static final double restingAngle = 0.0;

    public static final double FORWARD_LIMIT = 60.0;
    public static final double REVERSE_LIMIT = -60.0;

    public static final double turretAngleChute = 0.0;
    public static final double turretAngleShelf = 0.0;

    public static final double incrementAngle = 4.0;

    public static final class SimInfo {
      public static final double kV = 0.09402;
      public static final double kA = 0.033012;
    }
  }

  public static final class Swerve {
    public static final int imuCanID = 1;
    public static final double maxVelTele = 4.7;
    public static final double maxAccelTele = 6.0;
    public static final double maxAngularVelTele = Units.degreesToRadians(180);
    public static final double maxAngularAccelTele = Units.degreesToRadians(540);
    public static final double teleAngleHoldFactor = 3.0;

    public static final class Odometry {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
      public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
      public static final Matrix<N3, N1> visionStdDevsTrust = VecBuilder.fill(0.2, 0.2, 0.2);
    }

    public static final class PathFollowing {
      public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(2.0, 0.0, 0.0);
    }

    public static final class FrontLeftModule {
      public static final int driveMotorCanID = 7;
      public static final int rotationMotorCanID = 8;
      public static final int rotationEncoderCanID = 4;
      public static Translation2d moduleOffset =
          new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(24));
    }

    public static final class FrontRightModule {
      public static final int driveMotorCanID = 1;
      public static final int rotationMotorCanID = 2;
      public static final int rotationEncoderCanID = 1;
      public static Translation2d moduleOffset =
          new Translation2d(Units.inchesToMeters(24), -Units.inchesToMeters(24));
    }

    public static final class BackLeftModule {
      public static final int driveMotorCanID = 5;
      public static final int rotationMotorCanID = 6;
      public static final int rotationEncoderCanID = 3;
      public static Translation2d moduleOffset =
          new Translation2d(-Units.inchesToMeters(24), Units.inchesToMeters(24));
    }

    public static final class BackRightModule {
      public static final int driveMotorCanID = 3;
      public static final int rotationMotorCanID = 4;
      public static final int rotationEncoderCanID = 2;
      public static Translation2d moduleOffset =
          new Translation2d(-Units.inchesToMeters(24), -Units.inchesToMeters(24));
    }
  }

  public static final class AutoBalance {
    public static final PIDConstants BALANCE_CONSTANTS = new PIDConstants(0.3, 0.0, 0.1);
    public static final double maxVelAuto = 0.4;
    public static final double maxVelTele = 0.3;
  }

  public static final class LEDStrip {
    public static final int numLEDs = 45;

    public static final int candleID = 1;
    public static final int swerveLED = 0;
    public static final int pinkArmLED = 1;
    public static final int turretLED = 2;
    public static final int intakeLED = 3;
    public static final int jawLED = 4;
    public static final int stealerLED = 5;
    public static final int manhattanLED = 6;
  }

  public static final class Intake {
    public static final int intakeMotorID = 6;

    public static final int intakeCurrentLimit = 60;

    public static final double intakeHoldPercentCone = -0.1;
    public static final double intakeHoldPercentCube = 0.05;

    public static double intakeSpeedCone = -0.7;
    public static double intakeSpeedCube = 0.6;
    public static double outtakeSpeedCone = 0.6;
    public static double outtakeSpeedCube = -0.45;
    public static double outtakeSpeedCubeLow = -0.2;
  }

  public static final class Jaw {
    public static final int pivotMotorID = 5;

    public static final int pivotCurrentLimit = 15;

    public static final double pivotP = 0.07;
    public static final double pivotI = 0.0;
    public static final double pivotD = 0.0;

    public static final double gearing = 1.0 / 300.0;
    public static final double pivotDegreesPerPulse = gearing * 360.0;

    public static final double pivotCubeModePos = 120.0;
    public static final double pivotConePickupStandingPos = 55.0;
    public static final double pivotConePickupBehindPos = 340.0;
    public static final double pivotConePickupTippedPos = 0.0;
    public static final double pivotConeScoringPos = 65.0;
    public static final double pivotConeScoringPosLow = 0.0;
    public static final double pivotConeShelfPos = 125.0;
  }

  public static final class StealyWheely {
    public static final int motorID = 14;

    public static final int motorCurrentLimit = 20;

    public static final double intakeSpeed = 0.6;
    public static final double intakeHoldSpeed = 0.2;
    public static final double outtakeSpeed = -1.0;
  }

  public static final class Manhattan {
    public static final int armID = 40;
    public static final int armEncoderID = 40;
    public static final int tankID = 15;
    public static final int tankCurrentLimit = 80;
  }

  public static final class FieldProfiles {
    public static final FieldCalibration warehouseProfile =
        FieldCalibration.fromPoleOffsets(
            new Translation3d[] { // Blue pole offsets (top to bottom, driver left to right)
              Util.trans3dFromInches(-1.0, 0.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0),
              Util.trans3dFromInches(-1.0, -2.0, 0.0),
              Util.trans3dFromInches(-1.0, -1.0, 0.0),
              Util.trans3dFromInches(-1.0, -1.0, 0.0),
              Util.trans3dFromInches(-1.0, -2.0, 0.0),
              Util.trans3dFromInches(-1.0, -2.0, 0.0),
              Util.trans3dFromInches(-1.0, -3.0, 0.0)
            },
            new Translation3d[] { // Red pole offsets (top to bottom, driver left to right)
              Util.trans3dFromInches(0.5, 2.5, 0.0),
              Util.trans3dFromInches(0.5, 1.0, 0.0),
              Util.trans3dFromInches(-1.5, 1.0, 0.0),
              Util.trans3dFromInches(-1.0, 0.0, 0.0),
              Util.trans3dFromInches(-0.5, 0.5, 0.0),
              Util.trans3dFromInches(-1.5, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 0.0, 0.0),
              Util.trans3dFromInches(-1.0, 2.0, 0.0),
              Util.trans3dFromInches(-0.5, 3.0, 0.0),
              Util.trans3dFromInches(-0.0, 0.0, 0.0),
              Util.trans3dFromInches(0.5, -2.0, 0.0),
              Util.trans3dFromInches(-1.0, 0.5, 0.0)
            });

    public static final FieldCalibration champsProfile =
        FieldCalibration.fromPoleOffsets(
            new Translation3d[] { // Blue pole offsets (top to bottom, driver left to right)
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0)
            },
            new Translation3d[] { // Red pole offsets (top to bottom, driver left to right)
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0),
              Util.trans3dFromInches(0.0, 1.0, 0.0)
            });

    public static final FieldCalibration currentProfile = champsProfile;
  }
}
