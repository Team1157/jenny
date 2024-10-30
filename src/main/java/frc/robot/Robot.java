package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  double kP = 0.05;
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  @Override
  public void robotInit() {
    m_rightDrive.setInverted(false);
    gyro.calibrate(); // Calibrate gyro on startup

    // Add a toggle to the dashboard for selecting field-relative or robot-relative control
    SmartDashboard.putBoolean("Field-Relative Drive", true); // Default to field-relative
  }

  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Get the joystick inputs
    double xSpeed = -m_controller.getLeftY(); // Forward/Backward
    double ySpeed = m_controller.getLeftX(); // Left/Right

    // Check if field-relative mode is enabled
    boolean isFieldRelative = SmartDashboard.getBoolean("Field-Relative Drive", true);

    if (isFieldRelative) {
      // Calculate the field-relative angle
      double gyroAngle = Math.toRadians(gyro.getAngle()); // Convert to radians
      double temp = xSpeed * Math.cos(gyroAngle) + ySpeed * Math.sin(gyroAngle);
      ySpeed = -xSpeed * Math.sin(gyroAngle) + ySpeed * Math.cos(gyroAngle);
      xSpeed = temp;
    }

    // Use arcadeDrive with the transformed inputs if field-relative, or raw if robot-relative
    m_robotDrive.arcadeDrive(xSpeed, ySpeed);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
