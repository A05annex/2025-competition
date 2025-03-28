package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

@SuppressWarnings("unused")
public class SpinAlgaeWheelCommand extends Command {
	private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();

	public SpinAlgaeWheelCommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.algaeSubsystem);
	}

	@Override
	public void initialize() {
       algaeSubsystem.spin();
	}

	@Override
	public boolean isFinished() {
		return algaeSubsystem.getVelocity() > 50.0;
	}
}
