package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    private final Encoder m_leftEncoder = new Encoder(0, 1);
    private final Encoder m_rightEncoder = new Encoder(2, 3);
    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.762);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0.0, 0.0);
    
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);

    // NetworkTable entries
    private final NetworkTable m_telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry");
    private final NetworkTableEntry m_ntSpeedMultiplier = m_telemetryTable.getEntry("SpeedMultiplier");
    private final NetworkTableEntry m_ntAccelerationSetpoint = m_telemetryTable.getEntry("AccelerationSetpoint");
    private final NetworkTableEntry m_ntDriveMode = m_telemetryTable.getEntry("DriveMode"); // "arcade" or "tank"
    
    private double m_speedMultiplier = 3.0;
    private double m_accelerationSetpoint = 3.0;
    private String m_driveMode = "arcade";

    public Drivetrain() {
        // Initialize default NetworkTables values
        m_ntSpeedMultiplier.setDouble(m_speedMultiplier);
        m_ntAccelerationSetpoint.setDouble(m_accelerationSetpoint);
        m_ntDriveMode.setString(m_driveMode);
        
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_gyro.reset();
    }

    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(m_speedLimiter.calculate(fwd * m_speedMultiplier), rot * m_speedMultiplier);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(m_speedLimiter.calculate(leftSpeed * m_speedMultiplier), m_speedLimiter.calculate(rightSpeed * m_speedMultiplier));
    }

    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    @Override
    public void periodic() {
        // Update the speed multiplier and acceleration setpoint from NetworkTables
        m_speedMultiplier = m_ntSpeedMultiplier.getDouble(3.0);
        m_accelerationSetpoint = m_ntAccelerationSetpoint.getDouble(3.0);
        m_driveMode = m_ntDriveMode.getString("arcade");

        updateOdometry();
        
        SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
        SmartDashboard.putNumber("Left Encoder Distance", m_leftEncoder.getDistance());
        SmartDashboard.putNumber("Right Encoder Distance", m_rightEncoder.getDistance());
    }

    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // New method to get the current drive mode from NetworkTables
    public String getDriveMode() {
        return m_driveMode;
    }
}
