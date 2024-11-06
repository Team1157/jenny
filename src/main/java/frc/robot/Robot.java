package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    private final XboxController m_controller = new XboxController(0);
    private final Timer m_timer = new Timer();
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

    // Speed multiplier and acceleration PID constants 
    private double speedMultiplier;
    private double accelerationSetpoint;
    private double accelerationKP = 0.1;
    private double previousSpeed = 0.0;
    private double speed = 0.0;
    private double lastUpdateTime;

    // Variables for velocity and position tracking using accelerometer
    private double velocityX = 0.0;
    private double velocityY = 0.0;
    private double positionX = 0.0;
    private double positionY = 0.0;
    private final double dt = 0.02;  // Assuming a 20ms loop interval

    @Override
    public void robotInit() {
        m_rightDrive.setInverted(false);
        gyro.calibrate();
        
        SmartDashboard.putBoolean("Field-Relative Drive", false);
        SmartDashboard.putNumber("Speed Multiplier", 1.0);
        SmartDashboard.putNumber("Acceleration Setpoint", 0.2);

        lastUpdateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void teleopPeriodic() {
        speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 1.0);
        accelerationSetpoint = SmartDashboard.getNumber("Acceleration Setpoint", 0.2);

        double xSpeed = -m_controller.getLeftY() * speedMultiplier;
        double ySpeed = m_controller.getRawAxis(5) * speedMultiplier;

        double currentSpeed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        double acceleration = (currentSpeed - previousSpeed) / dt;

        double speedAdjustment = (accelerationSetpoint - acceleration) * accelerationKP;
        xSpeed += speedAdjustment;
        ySpeed += speedAdjustment;

        boolean isFieldRelative = SmartDashboard.getBoolean("Field-Relative Drive", true);
        if (isFieldRelative) {
            double gyroAngle = Math.toRadians(gyro.getAngle());
            double temp = xSpeed * Math.cos(gyroAngle) + ySpeed * Math.sin(gyroAngle);
            ySpeed = -xSpeed * Math.sin(gyroAngle) + ySpeed * Math.cos(gyroAngle);
            xSpeed = temp;
        }

        m_robotDrive.arcadeDrive(xSpeed, ySpeed);

        previousSpeed = currentSpeed;

        // Update velocity and position from accelerometer
        updateVelocityAndPosition();
        
        // Update the speedometer display
        updateSpeedometer();
    }

    private void updateVelocityAndPosition() {
        // Get acceleration values in m/s^2
        double accelX = accelerometer.getX() * 9.81; // Assuming accelerometer returns in G's, convert to m/s^2
        double accelY = accelerometer.getY() * 9.81;

        // Integrate acceleration to get velocity
        velocityX += accelX * dt;
        velocityY += accelY * dt;

        // Integrate velocity to get position
        positionX += velocityX * dt;
        positionY += velocityY * dt;
    }

    private void updateSpeedometer() {
        // Calculate the speed as the magnitude of velocity vector
        speed = Math.sqrt(velocityX * velocityX + velocityY * velocityY);

        SmartDashboard.putNumber("Robot Speed (m/s)", speed);
        SmartDashboard.putNumber("Robot Position X (m)", positionX);
        SmartDashboard.putNumber("Robot Position Y (m)", positionY);
    }
}
