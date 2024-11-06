package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    private final XboxController m_controller = new XboxController(0);
    private final Timer m_timer = new Timer();
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

    private double speedMultiplier;
    private double accelerationSetpoint;
    private double accelerationKP = 0.1;
    private double previousSpeed = 0.0;
    private double speed = 0.0;
    private double lastUpdateTime;

    // NetworkTables entries
    private NetworkTableEntry ntSpeed;
    private NetworkTableEntry ntAcceleration;
    private NetworkTableEntry ntGyroAngle;

    @Override
    public void robotInit() {
        // Camera setup for USB webcam
        CameraServer.startAutomaticCapture();

        // Initialize gyro and NetworkTables
        gyro.calibrate();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable telemetryTable = inst.getTable("Telemetry");

        // NetworkTables entries for telemetry data
        ntSpeed = telemetryTable.getEntry("RobotSpeed");
        ntAcceleration = telemetryTable.getEntry("RobotAcceleration");
        ntGyroAngle = telemetryTable.getEntry("GyroAngle");

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
        double acceleration = (currentSpeed - previousSpeed) / 0.02;

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
        updateTelemetry();
    }

    private void updateTelemetry() {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        double accelerationX = accelerometer.getX();
        double accelerationY = accelerometer.getY();

        double accelerationMagnitude = Math.sqrt(accelerationX * accelerationX + accelerationY * accelerationY);
        speed += accelerationMagnitude * deltaTime;

        // Publish data to NetworkTables
        ntSpeed.setDouble(speed);
        ntAcceleration.setDouble(accelerationMagnitude);
        ntGyroAngle.setDouble(gyro.getAngle());

        // Display on SmartDashboard
        SmartDashboard.putNumber("Robot Speed (m/s)", speed);
        SmartDashboard.putNumber("Robot Acceleration (m/s^2)", accelerationMagnitude);
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }
}
