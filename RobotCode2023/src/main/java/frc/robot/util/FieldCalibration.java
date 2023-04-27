package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldCalibration {
  private static final int ROWS = 3;
  private static final int COLS = 9;

  private final Translation3d[] scoringOffsetsBlue;
  private final Translation3d[] scoringOffsetsRed;

  public FieldCalibration() {
    scoringOffsetsBlue = new Translation3d[ROWS * COLS];
    scoringOffsetsRed = new Translation3d[ROWS * COLS];

    for (int i = 0; i < ROWS * COLS; i++) {
      scoringOffsetsBlue[i] = new Translation3d(0, 0, 0);
      scoringOffsetsRed[i] = new Translation3d(0, 0, 0);
    }
  }

  public FieldCalibration(Translation3d[] blueOffsets, Translation3d[] redOffsets) {
    if (blueOffsets.length != ROWS * COLS) {
      throw new IllegalArgumentException(
          "Blue offset array length not equal to number of scoring positions");
    }
    if (redOffsets.length != ROWS * COLS) {
      throw new IllegalArgumentException(
          "Red offset array length not equal to number of scoring positions");
    }

    scoringOffsetsBlue = blueOffsets;
    scoringOffsetsRed = redOffsets;
  }

  public static FieldCalibration fromPoleOffsets(
      Translation3d[] bluePoleOffsets, Translation3d[] redPoleOffsets) {
    if (bluePoleOffsets.length != (ROWS - 1) * (COLS - 3)) {
      throw new IllegalArgumentException(
          "Blue pole offset array length not equal to number of pole positions");
    }
    if (redPoleOffsets.length != (ROWS - 1) * (COLS - 3)) {
      throw new IllegalArgumentException(
          "Red pole offset array length not equal to number of pole positions");
    }

    Translation3d[] blueOffsets = new Translation3d[ROWS * COLS];
    Translation3d[] redOffsets = new Translation3d[ROWS * COLS];

    for (int i = 0; i < 3; i++) {
      // Top row
      blueOffsets[(i * 3)] = bluePoleOffsets[(i * 2)];
      blueOffsets[(i * 3) + 1] = new Translation3d(0, 0, 0);
      blueOffsets[(i * 3) + 2] = bluePoleOffsets[(i * 2) + 1];
      redOffsets[(i * 3)] = redPoleOffsets[(i * 2)];
      redOffsets[(i * 3) + 1] = new Translation3d(0, 0, 0);
      redOffsets[(i * 3) + 2] = redPoleOffsets[(i * 2) + 1];
      // Mid row
      blueOffsets[(i * 3) + COLS] = bluePoleOffsets[(i * 2) + (COLS - 3)];
      blueOffsets[(i * 3) + 1 + COLS] = new Translation3d(0, 0, 0);
      blueOffsets[(i * 3) + 2 + COLS] = bluePoleOffsets[(i * 2) + 1 + (COLS - 3)];
      redOffsets[(i * 3) + COLS] = redPoleOffsets[(i * 2) + (COLS - 3)];
      redOffsets[(i * 3) + 1 + COLS] = new Translation3d(0, 0, 0);
      redOffsets[(i * 3) + 2 + COLS] = redPoleOffsets[(i * 2) + 1 + (COLS - 3)];
      // Bottom row
      blueOffsets[(i * 3) + (2 * COLS)] = new Translation3d(0, 0, 0);
      blueOffsets[(i * 3) + 1 + (2 * COLS)] = new Translation3d(0, 0, 0);
      blueOffsets[(i * 3) + 2 + (2 * COLS)] = new Translation3d(0, 0, 0);
      redOffsets[(i * 3) + (2 * COLS)] = new Translation3d(0, 0, 0);
      redOffsets[(i * 3) + 1 + (2 * COLS)] = new Translation3d(0, 0, 0);
      redOffsets[(i * 3) + 2 + (2 * COLS)] = new Translation3d(0, 0, 0);
    }

    return new FieldCalibration(blueOffsets, redOffsets);
  }

  public Translation3d getScoringPositionOffset(
      int scoringPosIdx, DriverStation.Alliance allianceColor) {
    if (allianceColor == DriverStation.Alliance.Blue) {
      return scoringOffsetsBlue[scoringPosIdx];
    } else {
      return scoringOffsetsRed[scoringPosIdx];
    }
  }

  public Translation3d getScoringPositionOffset(
      int scoringPosRow, int scoringPosCol, DriverStation.Alliance allianceColor) {
    return getScoringPositionOffset((scoringPosRow * COLS) + scoringPosCol, allianceColor);
  }
}
