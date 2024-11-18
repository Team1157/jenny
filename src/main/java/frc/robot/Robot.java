package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class Robot extends TimedRobot {
    private final XboxController m_controller = new XboxController(0);
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    private final Encoder m_leftEncoder = new Encoder(0, 1);
    private final Encoder m_rightEncoder = new Encoder(2, 3);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter m_speedLimiter1 = new SlewRateLimiter(2);
    private final SlewRateLimiter m_speedLimiter2 = new SlewRateLimiter(2);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.762);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    private final LTVUnicycleController m_feedback = new LTVUnicycleController(0.020);
    private final Timer m_timer = new Timer();

    private final Field2d m_fieldSim = new Field2d();
    private final DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3),
            DCMotor.getKrakenX60Foc(2), 16, 3.2, .8, null);

    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    private NetworkTableEntry ntSpeed;
    private NetworkTableEntry ntAcceleration;
    private NetworkTableEntry ntGyroAngle;

    private NetworkTableEntry ntLeftStickAxis;
    private NetworkTableEntry ntRightXStickAxis;
    private NetworkTableEntry ntRightYStickAxis;

    private NetworkTableEntry ntSpeedMultiplier;
    private NetworkTableEntry ntAccelerationSetpoint;

    private double speed = 0.0;
    private double previousSpeed = 0.0;
    private final Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(6, 4, new Rotation2d(0.0)),
            new TrajectoryConfig(2, 2));

    // Add a chooser for selecting between arcade and differential drive
    private final SendableChooser<String> m_driveModeChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(0).setResolution(1280, 720);
        CameraServer.startAutomaticCapture(1).setResolution(1280, 720);
    

        NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry");
        ntSpeed = telemetryTable.getEntry("RobotSpeed");
        ntAcceleration = telemetryTable.getEntry("RobotAcceleration");
        ntGyroAngle = telemetryTable.getEntry("GyroAngle");

        // Initialize SmartDashboard settings
        SmartDashboard.putData("Field", m_fieldSim);
        m_driveModeChooser.setDefaultOption("Arcade Drive", "arcade");
        m_driveModeChooser.addOption("Differential Drive", "differential");
        SmartDashboard.putData("Drive Mode", m_driveModeChooser);

        ntLeftStickAxis = telemetryTable.getEntry("LeftStickAxis");
        ntRightXStickAxis = telemetryTable.getEntry("RightXStickAxis");
        ntRightYStickAxis = telemetryTable.getEntry("RightYStickAxis");
        ntSpeedMultiplier = telemetryTable.getEntry("SpeedMultiplier");
        ntAccelerationSetpoint = telemetryTable.getEntry("AccelerationSetpoint");

        ntLeftStickAxis.setDouble(1); // Default axis for left stick
        ntRightXStickAxis.setDouble(5); // Default axis for right stick
        ntRightYStickAxis.setDouble(4); // Default axis for right stick

        ntSpeedMultiplier.setDouble(1); // Default speed multiplier
        ntAccelerationSetpoint.setDouble(3.0); // Default acceleration setpoint
    }

    @Override
    public void robotPeriodic() {
        updateOdometry();
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Robot Speed (m/s)", speed);
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

    private void updateOdometry() {
        m_odometry.update(gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }
    
    @Override
    public void teleopPeriodic() {
        // Get selected joystick axes from NetworkTables
        int leftAxis = (int) ntLeftStickAxis.getDouble(1);
        int rightXAxis = (int) ntRightXStickAxis.getDouble(5);
        int rightYAxis = (int) ntRightYStickAxis.getDouble(5);
    
        // Get joystick values from selected axes
        double leftY = m_controller.getRawAxis(leftAxis);
        double rightX = m_controller.getRawAxis(rightXAxis);
        double rightY = -m_controller.getRawAxis(rightYAxis);
    
        // Get speed multiplier and acceleration setpoint from NetworkTables
        double speedMultiplier = ntSpeedMultiplier.getDouble(3.0);
    
        // Check if button 8 is pressed
        if (m_controller.getRawButton(8)) {
            // Apply rate limiters
            double leftVertSpeed = m_speedLimiter.calculate(-leftY * speedMultiplier);
            double rightHoriSpeed = m_speedLimiter1.calculate(rightX * speedMultiplier);
            double rightVertSpeed = m_speedLimiter2.calculate(rightY * speedMultiplier);
    
            // Set drive mode based on chooser
            String driveMode = m_driveModeChooser.getSelected();
            if (m_controller.getRawButton(2)) {
                gyro.reset();
            }
            if ("differential".equals(driveMode)) {
                m_robotDrive.tankDrive(leftVertSpeed, rightVertSpeed);
            } else if ("arcade".equals(driveMode)) {
                m_robotDrive.arcadeDrive(leftVertSpeed, rightHoriSpeed);
            }
    
            // Telemetry for speed and acceleration
            double currentSpeed = (leftVertSpeed + rightHoriSpeed) / 2;
            double acceleration = (currentSpeed - previousSpeed) / 0.02;
            previousSpeed = currentSpeed;
    
            ntSpeed.setDouble(currentSpeed);
            ntAcceleration.setDouble(acceleration);
            ntGyroAngle.setDouble(gyro.getAngle());
        } else {
            // Reset slew rate limiters and stop the robot
            m_speedLimiter.reset(0);
            m_speedLimiter1.reset(0);
            m_speedLimiter2.reset(0);
            m_robotDrive.stopMotor();
    
            // Reset telemetry values
            ntSpeed.setDouble(0);
            ntAcceleration.setDouble(0);
        }
    }
    

    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void autonomousPeriodic() {
        double elapsed = m_timer.get();
        Trajectory.State reference = m_trajectory.sample(elapsed);
        ChassisSpeeds speeds = m_feedback.calculate(m_odometry.getPoseMeters(), reference);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_robotDrive.tankDrive(wheelSpeeds.leftMetersPerSecond / 3.0, wheelSpeeds.rightMetersPerSecond / 3.0);
    }

    @Override
    public void simulationPeriodic() {
        m_drivetrainSim.setInputs(
                -m_leftDrive.get() * RobotController.getBatteryVoltage(),
                -m_rightDrive.get() * RobotController.getBatteryVoltage());
        m_drivetrainSim.update(0.020);

        m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
    }
}
