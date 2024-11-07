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
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.BuiltInAccelerometerSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    private final XboxController m_controller = new XboxController(0);
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

    // Simulation components
    private ADXRS450_GyroSim gyroSim;
    private BuiltInAccelerometerSim accelerometerSim;
    private PWMSim leftMotorSim;
    private PWMSim rightMotorSim;

    @Override
    public void robotInit() {
        // Camera setup for USB webcam
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        // Initialize gyro and NetworkTables
        gyro.calibrate();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable telemetryTable = inst.getTable("Telemetry");

        // NetworkTables entries for telemetry data
        ntSpeed = telemetryTable.getEntry("RobotSpeed");
        ntAcceleration = telemetryTable.getEntry("RobotAcceleration");
        ntGyroAngle = telemetryTable.getEntry("GyroAngle");

        SmartDashboard.putBoolean("Field-Relative Drive", false);
        SmartDashboard.putNumber("Speed Multiplier", 2.0);
        SmartDashboard.putNumber("Acceleration Setpoint", 0.2);

        lastUpdateTime = Timer.getFPGATimestamp();

        // Simulation setup
        if (isSimulation()) {
            gyroSim = new ADXRS450_GyroSim(gyro);
            accelerometerSim = new BuiltInAccelerometerSim();
            leftMotorSim = new PWMSim(m_leftDrive.getChannel());
            rightMotorSim = new PWMSim(m_rightDrive.getChannel());
        }
    }

    @Override
    public void teleopPeriodic() {
        speedMultiplier = SmartDashboard.getNumber("Speed Multiplier", 2.0);
        accelerationSetpoint = SmartDashboard.getNumber("Acceleration Setpoint", 0.2);

        double xSpeed = -m_controller.getLeftY() * speedMultiplier;
        double ySpeed = m_controller.getRawAxis(5) * 2 * speedMultiplier;

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

    @Override
    public void simulationPeriodic() {
        // Simulate basic drivetrain physics for gyro and accelerometer
        double leftSpeed = leftMotorSim.getSpeed();
        double rightSpeed = rightMotorSim.getSpeed();

        // Approximate linear velocity by averaging motor outputs (this is very basic)
        double avgSpeed = (leftSpeed + rightSpeed) / 2.0 * 3.0; // scaling factor for simulation
        speed = avgSpeed;

        // Simulate gyro angle change based on turn rate
        double turnRate = (rightSpeed - leftSpeed) * 45; // degrees per second approximation
        gyroSim.setAngle(gyro.getAngle() + turnRate * 0.02);

        // Simulate accelerometer readings (rough estimate based on motor output)
        accelerometerSim.setX(avgSpeed * 0.1);
        accelerometerSim.setY(0);
    }

    private void updateTelemetry() {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        double accelerationX = accelerometer.getY();
        double accelerationY = accelerometer.getX();

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
