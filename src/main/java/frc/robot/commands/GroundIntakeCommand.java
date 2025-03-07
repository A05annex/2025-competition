package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;


public class GroundIntakeCommand extends Command {
	@SuppressWarnings("FieldCanBeLocal")
	private final ElevatorSubsystem elevatorSubsystem;
	private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();
	private final GroundIntakeSubsystem groundIntakeSubsystem = GroundIntakeSubsystem.getInstance();

	public GroundIntakeCommand(ElevatorSubsystem elevatorSubsystem) {
		this.elevatorSubsystem = elevatorSubsystem;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(this.elevatorSubsystem, this.endEffectorSubsystem, this.groundIntakeSubsystem);
	}

	@Override
	public void initialize() {
		groundIntakeSubsystem.extendActuator();
		groundIntakeSubsystem.setIntakeVelocity(2000.0);
		ElevatorSubsystem.ELEVATOR_POSITION.AGI.goTo();
		endEffectorSubsystem.setVelocity(-1000.0);
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return Constants.frontSensor();
	}

	@Override
	public void end(boolean interrupted) {
		ElevatorSubsystem.ELEVATOR_POSITION.L1.goTo();
		endEffectorSubsystem.stopAll();
		groundIntakeSubsystem.retractActuator();
		groundIntakeSubsystem.stopAll();
	}
}
