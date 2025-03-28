package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

@SuppressWarnings("unused")
public class ReserveEECommand extends Command {
	@SuppressWarnings("FieldCanBeLocal")
	private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();

	public ReserveEECommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.endEffectorSubsystem);
	}
}
