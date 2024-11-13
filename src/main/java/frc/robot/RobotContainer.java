package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final XboxController m_controller = new XboxController(0);

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
    }

    private void configureBindings() {
        // No explicit binding required since we're using NetworkTables entries.
    }

    private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(new TeleopDriveCommand(m_drivetrain, m_controller));
    }
}
