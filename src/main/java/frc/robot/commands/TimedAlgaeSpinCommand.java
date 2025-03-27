package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class TimedAlgaeSpinCommand extends Command {
	private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();

	private int counter = 0;

	private final boolean forward;

	public TimedAlgaeSpinCommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.algaeSubsystem);
		forward = true;
	}

	public TimedAlgaeSpinCommand(Boolean forward) {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.algaeSubsystem);

		this.forward = forward;
	}

	@Override
	public void initialize() {
		if(forward) {
			algaeSubsystem.spin();
		} else {
			algaeSubsystem.reverse();
		}
		counter = 0;
	}

	@Override
	public void execute() {
		counter++;
	}

	@Override
	public boolean isFinished() {
		return forward ? counter > 50 : counter > 75;
	}

	@Override
	public void end(boolean interrupted) {
		algaeSubsystem.stop();
	}
}
