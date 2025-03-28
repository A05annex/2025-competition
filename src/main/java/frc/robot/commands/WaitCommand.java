package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


@SuppressWarnings("unused")
public class WaitCommand extends Command {
	private final int cycles;
	private int currentCycles;

	public WaitCommand(Integer cycles) {
		this.cycles = cycles;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements();
	}

	@Override
	public void initialize() {
		currentCycles = 0;
	}

	@Override
	public boolean isFinished() {
		currentCycles++;
		return currentCycles >= cycles;
	}
}
