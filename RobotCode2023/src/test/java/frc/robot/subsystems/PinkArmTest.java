package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class PinkArmTest {
  private static double DELTA = 0.01;

  @BeforeEach
  public void init() {
    HAL.initialize(500, 0);
  }

  //  @Test
  //  public void testSetGoalAngle() {
  //    PinkArm pinkArm = new PinkArm();
  //
  //    pinkArm.setJointGoalAngle(0);
  //    assertEquals(0, pinkArm.jointSetpoint, DELTA);
  //
  //    pinkArm.setJointGoalAngle(Constants.PinkArm.Joint.FORWARD_LIMIT * 0.8);
  //    assertEquals(Constants.PinkArm.Joint.FORWARD_LIMIT * 0.8, pinkArm.jointSetpoint, DELTA);
  //
  //    pinkArm.setJointGoalAngle(Constants.PinkArm.Joint.REVERSE_LIMIT * 0.8);
  //    assertEquals(Constants.PinkArm.Joint.REVERSE_LIMIT * 0.8, pinkArm.jointSetpoint, DELTA);
  //
  //    pinkArm.setJointGoalAngle(Constants.PinkArm.Joint.FORWARD_LIMIT * 1.5);
  //    assertEquals(Constants.PinkArm.Joint.FORWARD_LIMIT, pinkArm.jointSetpoint, DELTA);
  //
  //    pinkArm.setJointGoalAngle(Constants.PinkArm.Joint.REVERSE_LIMIT * 1.5);
  //    assertEquals(Constants.PinkArm.Joint.REVERSE_LIMIT, pinkArm.jointSetpoint, DELTA);
  //  }

  @Test
  public void testSetGoalExtension() {
    PinkArm pinkArm = new PinkArm();

    pinkArm.setExtensionGoalLength(0);
    assertEquals(0, pinkArm.extensionSetpoint, DELTA);

    pinkArm.setExtensionGoalLength(Constants.PinkArm.Extension.FORWARD_LIMIT * 0.8);
    assertEquals(Constants.PinkArm.Extension.FORWARD_LIMIT * 0.8, pinkArm.extensionSetpoint, DELTA);

    pinkArm.setExtensionGoalLength(Constants.PinkArm.Extension.REVERSE_LIMIT * 0.8);
    assertEquals(Constants.PinkArm.Extension.REVERSE_LIMIT * 0.8, pinkArm.extensionSetpoint, DELTA);

    pinkArm.setExtensionGoalLength(Constants.PinkArm.Extension.FORWARD_LIMIT * 1.5);
    assertEquals(Constants.PinkArm.Extension.FORWARD_LIMIT, pinkArm.extensionSetpoint, DELTA);

    pinkArm.setExtensionGoalLength(Constants.PinkArm.Extension.REVERSE_LIMIT * 1.5);
    assertEquals(Constants.PinkArm.Extension.REVERSE_LIMIT, pinkArm.extensionSetpoint, DELTA);
  }

  //  @Test
  //  public void testSetArmPositionCommand() {
  //    PinkArm pinkArm = new PinkArm();
  //
  //    CommandBase cmd = pinkArm.setArmPositionCommand(0, 0);
  //    cmd.initialize();
  //    assertEquals(0.0, pinkArm.extensionSetpoint, DELTA);
  //    assertEquals(0.0, pinkArm.jointSetpoint, DELTA);
  //
  //    cmd =
  //        pinkArm.setArmPositionCommand(
  //            Constants.PinkArm.Extension.FORWARD_LIMIT * 0.8,
  //            Constants.PinkArm.Joint.FORWARD_LIMIT * 0.8);
  //    cmd.execute();
  //    assertEquals(Constants.PinkArm.Extension.FORWARD_LIMIT * 0.8, pinkArm.extensionSetpoint,
  // DELTA);
  //    assertEquals(Constants.PinkArm.Joint.FORWARD_LIMIT * 0.8, pinkArm.jointSetpoint, DELTA);
  //
  //    cmd =
  //        pinkArm.setArmPositionCommand(
  //            Constants.PinkArm.Extension.FORWARD_LIMIT * 1.2,
  //            Constants.PinkArm.Joint.FORWARD_LIMIT * 1.2);
  //    cmd.execute();
  //    assertEquals(Constants.PinkArm.Extension.FORWARD_LIMIT, pinkArm.extensionSetpoint, DELTA);
  //    assertEquals(Constants.PinkArm.Joint.FORWARD_LIMIT, pinkArm.jointSetpoint, DELTA);
  //  }
}
