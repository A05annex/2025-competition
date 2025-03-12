package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class TimedAlgaeSpinCommand extends Command {
	private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();

	private int counter = 0;

	public TimedAlgaeSpinCommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.algaeSubsystem);
	}

	@Override
	public void initialize() {
		algaeSubsystem.spin();
		counter = 0;
	}

	@Override
	public void execute() {
		counter++;
	}

	@Override
	public boolean isFinished() {
		return counter > 25;
	}

	@Override
	public void end(boolean interrupted) {
		algaeSubsystem.stop();
	}
}
