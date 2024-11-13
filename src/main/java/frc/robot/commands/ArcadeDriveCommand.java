package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDriveCommand extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final XboxController m_controller;

    public ArcadeDriveCommand(Drivetrain drivetrain, XboxController controller) {
        m_drivetrain = drivetrain;
        m_controller = controller;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double forward = -m_controller.getLeftY(); // Negate if controller Y is inverted
        double rotation = m_controller.getRightX();
        m_drivetrain.arcadeDrive(forward, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
