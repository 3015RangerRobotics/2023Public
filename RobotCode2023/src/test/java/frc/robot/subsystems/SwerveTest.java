package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.swerve.Mk4SwerveModulePro;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Not really much that can be tested in here */
public class SwerveTest {
  private static final double DELTA = 1E-2;

  @BeforeEach
  public void init() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testModuleOrder() {
    Swerve swerve = new Swerve();

    assertEquals(4, swerve.modules.length);
    assertEquals(Mk4SwerveModulePro.ModuleCode.FL, swerve.modules[0].moduleCode);
    assertEquals(Mk4SwerveModulePro.ModuleCode.FR, swerve.modules[1].moduleCode);
    assertEquals(Mk4SwerveModulePro.ModuleCode.BL, swerve.modules[2].moduleCode);
    assertEquals(Mk4SwerveModulePro.ModuleCode.BR, swerve.modules[3].moduleCode);
  }

  @Test
  public void testResetOdom() {
    Swerve swerve = new Swerve();

    swerve.resetOdometry(new Pose2d());
    assertEquals(0.0, swerve.getPose().getTranslation().getX(), DELTA);
    assertEquals(0.0, swerve.getPose().getTranslation().getY(), DELTA);
    assertEquals(0.0, swerve.getPose().getRotation().getDegrees(), DELTA);

    swerve.resetOdometry(new Pose2d(1, 2, Rotation2d.fromDegrees(3)));
    assertEquals(1.0, swerve.getPose().getTranslation().getX(), DELTA);
    assertEquals(2.0, swerve.getPose().getTranslation().getY(), DELTA);
    assertEquals(3.0, swerve.getPose().getRotation().getDegrees(), DELTA);
  }
}
