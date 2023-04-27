package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CycleTracker {
  protected static Timer timer = new Timer();
  protected static StatCalculator stats = new StatCalculator();

  public static void trackCycle() {
    double cycleTime = timer.get();
    if (cycleTime != 0) {
      stats.addNumber(cycleTime);
      SmartDashboard.putNumber("CycleTracker/LastCycleTime", cycleTime);
      SmartDashboard.putNumber("CycleTracker/AvgCycleTime", stats.getMean());
      SmartDashboard.putNumber("CycleTracker/MinCycleTime", stats.getLowestValue());
      SmartDashboard.putNumber("CycleTracker/MaxCycleTime", stats.getHighestValue());
    }
    timer.reset();
    timer.start();
  }
}
