package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class MovingAverageTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testMovingAverage() {
    MovingAverage movingAverage = new MovingAverage(5);

    assertTrue(Double.isNaN(movingAverage.getAverage()));

    movingAverage.add(5);
    movingAverage.add(10);
    movingAverage.add(15);

    assertEquals(10, movingAverage.getAverage(), DELTA);

    movingAverage.add(100);

    assertEquals(32.5, movingAverage.getAverage(), DELTA);

    movingAverage.add(100);
    movingAverage.add(100);
    movingAverage.add(100);
    movingAverage.add(100);

    assertEquals(100, movingAverage.getAverage(), DELTA);
  }
}
