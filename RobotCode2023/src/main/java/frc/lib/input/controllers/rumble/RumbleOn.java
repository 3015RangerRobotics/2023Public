package frc.lib.input.controllers.rumble;

public class RumbleOn extends RumbleAnimation {
  @Override
  public double getRumbleOutput(double timeSeconds) {
    return 1.0;
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof RumbleOn;
  }
}
