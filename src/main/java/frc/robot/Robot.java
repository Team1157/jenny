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

    // Speed multiplier and acceleration PID constants
    private double speedMultiplier;
    private double accelerationSetpoint;
    private double accelerationKP = 0.1;  // Proportional constant for acceleration PID
    private double previousSpeed = 0.0;   // Used for calculating acceleration

    public Robot() {
        SendableRegistry.addChild(m_robotDrive, m_leftDrive);
        SendableRegistry.addChild(m_robotDrive, m_rightDrive);
    }

    @Override
    public void robotInit() {
        m_rightDrive.setInverted(false);
        gyro.calibrate(); // Calibrate gyro on startup

        // Add a toggle to the dashboard for selecting field-relative or robot-relative control
        SmartDashboard.putBoolean("Field-Relative Drive", false); // Default to non field-relative,  i don't want to like kill kieran

        // Add speed multiplier and acceleration setpoint to the SmartDashboard for adjustments
        SmartDashboard.putNumber("Speed Multiplier", 1.0);
        SmartDashboard.putNumber("Acceleration Setpoint", 0.2);  // Target acceleration in units/s^2 (1 is full 0 is stopped -1 is full reverse)
    }

    @Override
    public void autonomousInit() {
        m_timer.restart();
    }

    @Override
    public void autonomousPeriodic() {
		
    }

    @Override
    public void teleopPeriodic() {
        // Retrieve the multiplier and acceleration values from the SmartDashboard
        speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);
        accelerationSetpoint = SmartDashboard.getNumber("Acceleration Setpoint", 0.2);

        // Get the joystick inputs
        double xSpeed = -m_controller.getLeftY() * speedMultiplier; // Forward/Backward
        double ySpeed = m_controller.getRawAxis(0) * speedMultiplier;  // Left/Right

        // Calculate the current acceleration based on speed changes
        double currentSpeed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        double acceleration = (currentSpeed - previousSpeed) / 0.02; // 0.02 is the periodic interval

        // Adjust speed based on the difference between current acceleration and target setpoint
        double speedAdjustment = (accelerationSetpoint - acceleration) * accelerationKP;
        xSpeed += speedAdjustment;
        ySpeed += speedAdjustment;
        if (m_controller.getRawButton(2)) {
          gyro.reset();
        }
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

        // Update the previous speed for next acceleration calculation
        previousSpeed = currentSpeed;
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
