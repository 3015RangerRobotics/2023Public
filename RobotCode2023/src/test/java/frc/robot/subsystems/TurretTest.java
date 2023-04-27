package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TurretTest {
  private static double DELTA = 0.01;

  @BeforeEach
  public void init() {
    HAL.initialize(500, 0);
  }

  @Test
  public void setDesiredAngle() {
    Turret turret = new Turret();

    turret.setDesiredAngle(0);
    assertEquals(0, turret.turretSetpointDegrees, DELTA);

    turret.setDesiredAngle(Constants.Turret.FORWARD_LIMIT * 0.8);
    assertEquals(Constants.Turret.FORWARD_LIMIT * 0.8, turret.turretSetpointDegrees, DELTA);

    turret.setDesiredAngle(Constants.Turret.REVERSE_LIMIT * 0.8);
    assertEquals(Constants.Turret.REVERSE_LIMIT * 0.8, turret.turretSetpointDegrees, DELTA);
  }

  @Test
  public void testSetTurretAngleCommand() {
    Turret turret = new Turret();

    CommandBase cmd = turret.setTurretAngleCommand(0.0);
    cmd.execute();
    assertEquals(0.0, turret.turretSetpointDegrees, DELTA);

    cmd = turret.setTurretAngleCommand(Constants.Turret.FORWARD_LIMIT * 0.8);
    cmd.execute();
    assertEquals(Constants.Turret.FORWARD_LIMIT * 0.8, turret.turretSetpointDegrees, DELTA);

    cmd = turret.setTurretAngleCommand(Constants.Turret.REVERSE_LIMIT * 0.8);
    cmd.execute();
    assertEquals(Constants.Turret.REVERSE_LIMIT * 0.8, turret.turretSetpointDegrees, DELTA);
  }

  @Test
  public void testTurretRestingPosCommand() {
    Turret turret = new Turret();

    turret.setDesiredAngle(10.0);

    CommandBase cmd = turret.turretRestingCommand();
    cmd.execute();
    assertEquals(Constants.Turret.restingAngle, turret.turretSetpointDegrees, DELTA);
  }
}
