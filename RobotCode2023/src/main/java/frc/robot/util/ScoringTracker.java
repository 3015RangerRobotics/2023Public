package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Optional;

public class ScoringTracker {
  private static final int rows = 3;
  private static final int cols = 9;

  private static final double MID_POLE_Z = Units.inchesToMeters(34.0);
  private static final double MID_POLE_X = Units.inchesToMeters(31.35);

  private static final double HIGH_POLE_Z = Units.inchesToMeters(46.0);
  private static final double HIGH_POLE_X = Units.inchesToMeters(14.32);

  private static final double MID_SHELF_Z = Units.inchesToMeters(20.0);
  private static final double MID_SHELF_X = Units.inchesToMeters(31.0);

  private static final double HIGH_SHELF_Z = Units.inchesToMeters(34.0);
  private static final double HIGH_SHELF_X = Units.inchesToMeters(14.0);

  private static final double LOW_Z = 0.0;
  private static final double LOW_X = Units.inchesToMeters(40.0);

  private static final double FIELD_WIDTH = 8.02;

  private static final double COL_0_Y = FIELD_WIDTH - Units.inchesToMeters(20.13);
  private static final double COL_1_Y = FIELD_WIDTH - Units.inchesToMeters(42.13);
  private static final double COL_2_Y = FIELD_WIDTH - Units.inchesToMeters(64.13);
  private static final double COL_3_Y = FIELD_WIDTH - Units.inchesToMeters(86.13);
  private static final double COL_4_Y = FIELD_WIDTH - Units.inchesToMeters(108.13);
  private static final double COL_5_Y = FIELD_WIDTH - Units.inchesToMeters(130.13);
  private static final double COL_6_Y = FIELD_WIDTH - Units.inchesToMeters(152.13);
  private static final double COL_7_Y = FIELD_WIDTH - Units.inchesToMeters(174.13);
  private static final double COL_8_Y = FIELD_WIDTH - Units.inchesToMeters(196.13);

  private static final double BLUE_OFFSET = Units.inchesToMeters(99.07);

  // Ordered based on driver perspective. Top row -> mid row -> floor row
  // Starting at top left from driver perspective, top right from robot
  // Red alliance positions, these will be transformed for the blue alliance
  private static final Translation3d[] scorePositions =
      new Translation3d[] {
        // Top row
        new Translation3d(HIGH_POLE_X, COL_0_Y, HIGH_POLE_Z),
        new Translation3d(HIGH_SHELF_X, COL_1_Y, HIGH_SHELF_Z),
        new Translation3d(HIGH_POLE_X, COL_2_Y, HIGH_POLE_Z),
        new Translation3d(HIGH_POLE_X, COL_3_Y, HIGH_POLE_Z),
        new Translation3d(HIGH_SHELF_X, COL_4_Y, HIGH_SHELF_Z),
        new Translation3d(HIGH_POLE_X, COL_5_Y, HIGH_POLE_Z),
        new Translation3d(HIGH_POLE_X, COL_6_Y, HIGH_POLE_Z),
        new Translation3d(HIGH_SHELF_X, COL_7_Y, HIGH_SHELF_Z),
        new Translation3d(HIGH_POLE_X, COL_8_Y, HIGH_POLE_Z),
        // Mid row
        new Translation3d(MID_POLE_X, COL_0_Y, MID_POLE_Z),
        new Translation3d(MID_SHELF_X, COL_1_Y, MID_SHELF_Z),
        new Translation3d(MID_POLE_X, COL_2_Y, MID_POLE_Z),
        new Translation3d(MID_POLE_X, COL_3_Y, MID_POLE_Z),
        new Translation3d(MID_SHELF_X, COL_4_Y, MID_SHELF_Z),
        new Translation3d(MID_POLE_X, COL_5_Y, MID_POLE_Z),
        new Translation3d(MID_POLE_X, COL_6_Y, MID_POLE_Z),
        new Translation3d(MID_SHELF_X, COL_7_Y, MID_SHELF_Z),
        new Translation3d(MID_POLE_X, COL_8_Y, MID_POLE_Z),
        // Low row
        new Translation3d(LOW_X, COL_0_Y, LOW_Z),
        new Translation3d(LOW_X, COL_1_Y, LOW_Z),
        new Translation3d(LOW_X, COL_2_Y, LOW_Z),
        new Translation3d(LOW_X, COL_3_Y, LOW_Z),
        new Translation3d(LOW_X, COL_4_Y, LOW_Z),
        new Translation3d(LOW_X, COL_5_Y, LOW_Z),
        new Translation3d(LOW_X, COL_6_Y, LOW_Z),
        new Translation3d(LOW_X, COL_7_Y, LOW_Z),
        new Translation3d(LOW_X, COL_8_Y, LOW_Z),
      };

  public static boolean[] getScoring() {
    return SmartDashboard.getBooleanArray("ScoringTracker", new boolean[rows * cols]);
  }

  /**
   * @param r row of the scoring position
   * @param c column of the scoring position
   * @return whether or not the scoring position has been scored or not
   */
  public static boolean isScored(int r, int c) {
    boolean[] scored = getScoring();
    int pos = (r * cols) + c;
    return scored[pos];
  }

  /**
   * @param r row of scoring position
   * @param c column of scoring position
   * @param isScored value that we intend to change the value to
   */
  public static void setScored(int r, int c, boolean isScored) {
    // get array
    boolean[] scored = getScoring();
    int pos = (r * cols) + c;
    scored[pos] = isScored;
    // put it back
    SmartDashboard.putBooleanArray("ScoringTracker", scored);
  }

  /**
   * Method to get the best scoring position available
   *
   * @return the best scoring position available
   */
  public static ArrayList<Pair<Integer, Integer>> getBestScoringPositions() {
    boolean[] scored = getScoring();
    ArrayList<Pair<Integer, Integer>> ret = new ArrayList<>();
    // Loop through each row
    for (int row = 0; row < rows; row++) {
      // Loop through each col
      for (int col = 0; col < cols; col++) {
        int pos = (row * cols) + col;

        // Add this position to list if not scored
        if (!scored[pos]) {
          ret.add(new Pair<>(row, col));
        }
      }
      // Return the list if any positions from this row have been added
      if (!ret.isEmpty()) {
        return ret;
      }
    }
    return ret;
  }

  /**
   * @param r which row in the scoring grid is being located
   * @param c which column in the scoring grid is being located
   * @return Translation 3d that gives us coordinates of a scoring position
   */
  public static Translation3d getScoringPos(int r, int c) {
    int scoreIdx = (r * cols) + c;
    return getScoringPos(scoreIdx);
  }

  public static Translation3d getScoringPos(int idx) {
    Translation3d scorePos = scorePositions[idx];

    DriverStation.Alliance currentAlliance = DriverStation.getAlliance();
    if (currentAlliance == DriverStation.Alliance.Blue) {
      scorePos = new Translation3d(scorePos.getX(), scorePos.getY() - BLUE_OFFSET, scorePos.getZ());
    }

    return scorePos.plus(
        Constants.FieldProfiles.currentProfile.getScoringPositionOffset(idx, currentAlliance));
  }

  public static Translation3d getScoringPosAlliance(int r, int c, DriverStation.Alliance alliance) {
    int scoreIdx = (r * cols) + c;
    return getScoringPos(scoreIdx, alliance);
  }

  public static Translation3d getScoringPos(int idx, DriverStation.Alliance alliance) {
    Translation3d scorePos = scorePositions[idx];

    if (alliance == DriverStation.Alliance.Blue) {
      scorePos = new Translation3d(scorePos.getX(), scorePos.getY() - BLUE_OFFSET, scorePos.getZ());
    }

    return scorePos.plus(
        Constants.FieldProfiles.currentProfile.getScoringPositionOffset(idx, alliance));
  }

  /**
   * Get the closest, best scoring position based on current position of the robot
   *
   * @return Optional Translation3d representing the scoring position. Empty if no open spaces close
   *     enough
   */
  public static Optional<Translation3d> getClosestScoringPosition(Pose2d robotPose) {
    boolean[] scored = getScoring();

    boolean cubeMode = RobotContainer.isCubeMode();

    // Find the closest column to the robot
    int closestCol = cubeMode ? 1 : 0;
    for (int c = 2; c < cols; c++) {
      if ((cubeMode && (c == 4 || c == 7)) || (!cubeMode && (c != 4 && c != 7))) {
        if (Math.abs(getScoringPos(2, c).getY() - robotPose.getY())
            < Math.abs(getScoringPos(2, closestCol).getY() - robotPose.getY())) {
          closestCol = c;
        }
      }
    }

    // Loop through each row to find the best position to score
    for (int row = 0; row < 2; row++) {
      // First check the closest col
      if (!scored[(row * cols) + closestCol]) {
        return Optional.of(getScoringPos(row, closestCol));
      }

      // Then check neighboring cols if cone mode
      if (!cubeMode) {
        if ((closestCol == 3 || closestCol == 6) && !scored[(row * cols) + (closestCol - 1)]) {
          return Optional.of(getScoringPos(row, closestCol - 1));
        }

        if ((closestCol == 2 || closestCol == 5) && !scored[(row * cols) + (closestCol + 1)]) {
          return Optional.of(getScoringPos(row, closestCol + 1));
        }
      }
    }

    // If we haven't found anything yet, go for a floor position
    if (!scored[(2 * cols) + closestCol]) {
      return Optional.of(getScoringPos(2, closestCol));
    } else if (closestCol != 0 && !scored[(2 * cols) + closestCol - 1]) {
      return Optional.of(getScoringPos(2, closestCol - 1));
    } else if (closestCol != 8 && !scored[(2 * cols) + closestCol + 1]) {
      return Optional.of(getScoringPos(2, closestCol + 1));
    }

    return Optional.empty();
  }

  /**
   * Method to check if there is an override score position
   *
   * @return the value of the override input by the driver, or empty if it doesn't exist
   */
  public static Optional<Integer> getScoringTrackerOverride() {
    int override = (int) SmartDashboard.getNumber("ScoringTrackerOverride", -1);

    if (override == -1) {
      return Optional.empty();
    } else {
      return Optional.of(override);
    }
  }

  public static Optional<Translation3d> getScoringTrackerOverridePosition() {
    return getScoringTrackerOverride().map(ScoringTracker::getScoringPos);
  }

  /**
   * Get the scoring position that the robot should be aiming for, based on input from drivers and
   * available positions.
   *
   * @return Override position if there is one, or the best scoring position if no override is
   *     present.
   */
  public static Optional<Translation3d> getPreferredScoringPos(Pose2d robotPose) {
    return getScoringTrackerOverridePosition().or(() -> getClosestScoringPosition(robotPose));
  }
}
