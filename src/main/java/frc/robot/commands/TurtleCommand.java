package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;


public class TurtleCommand extends Command {
	private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
	private final GroundIntakeSubsystem groundIntakeSubsystem = GroundIntakeSubsystem.getInstance();

	public TurtleCommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.elevatorSubsystem, this.groundIntakeSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		ElevatorSubsystem.ELEVATOR_POSITION.SAFE.goTo();
		groundIntakeSubsystem.stopSpin();
		groundIntakeSubsystem.retractActuator();
	}
}
