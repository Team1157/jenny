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

    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.762);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    private final LTVUnicycleController m_feedback = new LTVUnicycleController(0.020);
    private final Timer m_timer = new Timer();

    private final Field2d m_fieldSim = new Field2d();
    private final DifferentialDrivetrainSim m_drivetrainSim = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3),
            DCMotor.getCIM(2), 8, 0.762, 0.0508, null);

    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    private NetworkTableEntry ntSpeed;
    private NetworkTableEntry ntAcceleration;
    private NetworkTableEntry ntGyroAngle;

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
        CameraServer.startAutomaticCapture(0);

        NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry");
        ntSpeed = telemetryTable.getEntry("RobotSpeed");
        ntAcceleration = telemetryTable.getEntry("RobotAcceleration");
        ntGyroAngle = telemetryTable.getEntry("GyroAngle");

        SmartDashboard.putData("Field", m_fieldSim);

        // Initialize the drive mode chooser
        m_driveModeChooser.setDefaultOption("Arcade Drive", "arcade");
        m_driveModeChooser.addOption("Differential Drive", "differential");
        SmartDashboard.putData("Drive Mode", m_driveModeChooser);

        // Show joystick axes on the SmartDashboard
        SmartDashboard.putNumber("Left Joystick Y Axis", 0.0);
        SmartDashboard.putNumber("Right Joystick X Axis", 0.0);
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
        // Get joystick values
        double leftY = m_controller.getLeftY();
        double rightX = -m_controller.getRawAxis(5);

        // Update the SmartDashboard with the joystick values
        SmartDashboard.putNumber("Left Joystick Y Axis", leftY);
        SmartDashboard.putNumber("Right Joystick X Axis", rightX);

        // Get the selected drive mode from the chooser
        String driveMode = m_driveModeChooser.getSelected();

        double xSpeed = -m_speedLimiter.calculate(leftY) * 3.0;
        double rot = -m_rotLimiter.calculate(rightX) * Math.PI;

        // Drive the robot based on the selected mode
        if ("arcade".equals(driveMode)) {
            m_robotDrive.arcadeDrive(xSpeed, rot);
        } else if ("differential".equals(driveMode)) {
            double leftSpeed = xSpeed + rot;
            double rightSpeed = xSpeed - rot;
            m_robotDrive.tankDrive(leftSpeed / 3.0, rightSpeed / 3.0);
        }

        // Calculate and send speed and acceleration telemetry data
        double currentSpeed = Math.hypot(xSpeed, rot);
        double acceleration = (currentSpeed - previousSpeed) / 0.02;
        previousSpeed = currentSpeed;

        ntSpeed.setDouble(currentSpeed);
        ntAcceleration.setDouble(acceleration);
        ntGyroAngle.setDouble(gyro.getAngle());
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
                m_leftDrive.get() * RobotController.getBatteryVoltage(),
                m_rightDrive.get() * RobotController.getBatteryVoltage());
        m_drivetrainSim.update(0.02);

        m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
    }
}
