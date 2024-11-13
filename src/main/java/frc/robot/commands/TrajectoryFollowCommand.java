package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollowCommand extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final Trajectory m_trajectory;
    private final Timer m_timer = new Timer();

    public TrajectoryFollowCommand(Drivetrain drivetrain, Trajectory trajectory) {
        m_drivetrain = drivetrain;
        m_trajectory = trajectory;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double elapsed = m_timer.get();
        // Implement trajectory following logic
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_drivetrain.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_trajectory.getTotalTimeSeconds();
    }
}
