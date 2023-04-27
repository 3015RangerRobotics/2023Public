package frc.lib.util;

import edu.wpi.first.util.CircularBuffer;

public class MovingAverage {
  private final CircularBuffer buffer;

  public MovingAverage(int size) {
    buffer = new CircularBuffer(size);
  }

  public void add(double d) {
    buffer.addLast(d);
  }

  public double getAverage() {
    if (buffer.size() == 0) {
      return Double.NaN;
    } else {
      double sum = 0;

      for (int i = 0; i < buffer.size(); i++) {
        sum += buffer.get(i);
      }

      return sum / buffer.size();
    }
  }
}
