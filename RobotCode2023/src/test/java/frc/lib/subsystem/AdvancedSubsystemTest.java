package frc.lib.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceLock;

public class AdvancedSubsystemTest {
  private static final double DELTA = 1E-2;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  public void cleanup() {
    SimHooks.resumeTiming();
  }

  @Test
  public void testName() {
    TestImpl test1 = new TestImpl();

    assertEquals("TestImpl", test1.getName());

    TestImpl test2 = new TestImpl("Test 2");

    assertEquals("Test 2", test2.getName());
  }

  // @Test
  // public void testBatteryUsageReporting() {
  //   RoboRioSim.setVInVoltage(12.0);
  //   BatteryUsage.reset();
  //   TestImpl sub = new TestImpl();

  //   assertEquals(0.0, BatteryUsage.getTotalUsage(), DELTA);

  //   sub.reportPowerUsage("TestDevice", 60, 12);
  //   assertEquals(0.01, BatteryUsage.getTotalUsage(), DELTA);

  //   sub.reportPowerUsage("TestDevice2", 600, 12);
  //   assertEquals(0.044, BatteryUsage.getTotalUsage(), DELTA);
  // }

  @Test
  @ResourceLock("timing")
  public void testSystemFault() {
    SimHooks.restartTiming();
    SubsystemFault fault = new SubsystemFault("test");

    assertEquals("test", fault.description);
    assertFalse(fault.isWarning);
    assertEquals(0.0, fault.timestamp, DELTA);

    SimHooks.stepTiming(1.0);
    fault = new SubsystemFault("test2", true);

    assertEquals("test2", fault.description);
    assertTrue(fault.isWarning);
    assertEquals(1.0, fault.timestamp, DELTA);
  }

  @Test
  public void testSystemFaultEquality() {
    SubsystemFault fault1 = new SubsystemFault("test");
    SubsystemFault fault2 = new SubsystemFault("test", true);
    SubsystemFault fault3 = new SubsystemFault("test", false);
    SubsystemFault fault4 = new SubsystemFault("test2");

    assertEquals(fault1, fault1);
    assertEquals(fault1, fault3);
    assertNotEquals(fault1, fault2);
    assertNotEquals(fault1, fault4);
    assertNotEquals(fault1, null);
  }

  @Test
  public void testAddFault() {
    TestImpl sub = new TestImpl();

    assertEquals(AdvancedSubsystem.SystemStatus.OK, sub.getSystemStatus());

    sub.addFault("test", true);

    assertEquals(1, sub.getFaults().size());
    assertEquals(AdvancedSubsystem.SystemStatus.WARNING, sub.getSystemStatus());

    sub.addFault("test2");

    assertEquals(2, sub.getFaults().size());
    assertEquals(AdvancedSubsystem.SystemStatus.ERROR, sub.getSystemStatus());

    sub.clearFaults();
    assertEquals(0, sub.getFaults().size());
  }

  @Test
  public void testAddFaultOnce() {
    TestImpl sub = new TestImpl();

    sub.addFault("test");
    assertEquals(1, sub.getFaults().size());

    sub.addFault("test");
    assertEquals(1, sub.getFaults().size());

    sub.addFault("test2", true);
    assertEquals(2, sub.getFaults().size());
  }

  public static class TestImpl extends AdvancedSubsystem {
    public TestImpl() {
      super();
    }

    public TestImpl(String name) {
      super(name);
    }

    @Override
    public void periodic() {}

    @Override
    public CommandBase systemCheckCommand() {
      return Commands.none();
    }
  }
}
