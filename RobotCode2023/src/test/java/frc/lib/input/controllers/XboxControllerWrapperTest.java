package frc.lib.input.controllers;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class XboxControllerWrapperTest {
  private static final double DELTA = 1E-2;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testButtons() {
    XboxControllerWrapper controller = new XboxControllerWrapper(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    assertFalse(controller.A().getAsBoolean());
    controllerSim.setAButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.A().getAsBoolean());

    assertFalse(controller.B().getAsBoolean());
    controllerSim.setBButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.B().getAsBoolean());

    assertFalse(controller.X().getAsBoolean());
    controllerSim.setXButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.X().getAsBoolean());

    assertFalse(controller.Y().getAsBoolean());
    controllerSim.setYButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.Y().getAsBoolean());

    assertFalse(controller.LB().getAsBoolean());
    controllerSim.setLeftBumper(true);
    controllerSim.notifyNewData();
    assertTrue(controller.LB().getAsBoolean());

    assertFalse(controller.RB().getAsBoolean());
    controllerSim.setRightBumper(true);
    controllerSim.notifyNewData();
    assertTrue(controller.RB().getAsBoolean());

    assertFalse(controller.LS().getAsBoolean());
    controllerSim.setLeftStickButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.LS().getAsBoolean());

    assertFalse(controller.RS().getAsBoolean());
    controllerSim.setRightStickButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.RS().getAsBoolean());

    assertFalse(controller.START().getAsBoolean());
    controllerSim.setStartButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.START().getAsBoolean());

    assertFalse(controller.BACK().getAsBoolean());
    controllerSim.setBackButton(true);
    controllerSim.notifyNewData();
    assertTrue(controller.BACK().getAsBoolean());
  }

  @Test
  public void testDPadButtons() {
    XboxControllerWrapper controller = new XboxControllerWrapper(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    assertFalse(controller.DUp().getAsBoolean());
    controllerSim.setPOV(0);
    controllerSim.notifyNewData();
    assertTrue(controller.DUp().getAsBoolean());

    assertFalse(controller.DRight().getAsBoolean());
    controllerSim.setPOV(90);
    controllerSim.notifyNewData();
    assertTrue(controller.DRight().getAsBoolean());

    assertFalse(controller.DDown().getAsBoolean());
    controllerSim.setPOV(180);
    controllerSim.notifyNewData();
    assertTrue(controller.DDown().getAsBoolean());

    assertFalse(controller.DLeft().getAsBoolean());
    controllerSim.setPOV(270);
    controllerSim.notifyNewData();
    assertTrue(controller.DLeft().getAsBoolean());
  }

  @Test
  public void testTriggerButtons() {
    XboxControllerWrapper controller = new XboxControllerWrapper(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    assertFalse(controller.LT().getAsBoolean());
    controllerSim.setLeftTriggerAxis(0.2);
    controllerSim.notifyNewData();
    assertTrue(controller.LT().getAsBoolean());

    assertFalse(controller.RT().getAsBoolean());
    controllerSim.setRightTriggerAxis(0.2);
    controllerSim.notifyNewData();
    assertTrue(controller.RT().getAsBoolean());
  }

  @Test
  public void testSticks() {
    XboxControllerWrapper controller = new XboxControllerWrapper(0, 0.2);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    controllerSim.setLeftX(0.5);
    controllerSim.setLeftY(-0.3);
    controllerSim.setRightX(-0.9);
    controllerSim.setRightY(1.0);
    controllerSim.notifyNewData();

    assertEquals(0.37, controller.getLeftX(), DELTA);
    assertEquals(-0.13, controller.getLeftY(), DELTA);
    assertEquals(-0.87, controller.getRightX(), DELTA);
    assertEquals(1.0, controller.getRightY(), DELTA);

    controllerSim.setLeftX(0.1);
    controllerSim.setLeftY(-0.05);
    controllerSim.setRightX(-0.13);
    controllerSim.setRightY(0.0);
    controllerSim.notifyNewData();

    assertEquals(0, controller.getLeftX(), DELTA);
    assertEquals(0, controller.getLeftY(), DELTA);
    assertEquals(0, controller.getRightX(), DELTA);
    assertEquals(0, controller.getRightY(), DELTA);
  }
}
