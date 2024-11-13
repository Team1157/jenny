package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final XboxController m_controller;

    public TeleopDriveCommand(Drivetrain drivetrain, XboxController controller) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        // Read current drive mode from NetworkTables
        String driveMode = m_drivetrain.getDriveMode();

        if ("arcade".equals(driveMode)) {
            double forward = m_controller.getLeftY(); 
            double rotation = -m_controller.getRawAxis(5);
            m_drivetrain.arcadeDrive(forward, rotation);
        } else if ("tank".equals(driveMode)) {
            double leftSpeed = m_controller.getLeftY();
            double rightSpeed = m_controller.getRightY();
            m_drivetrain.tankDrive(leftSpeed, rightSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
