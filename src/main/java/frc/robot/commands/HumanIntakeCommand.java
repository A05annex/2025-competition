package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class HumanIntakeCommand extends Command {

	ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
	EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();

	private int coveredTimer;

	public HumanIntakeCommand() {
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(elevatorSubsystem, endEffectorSubsystem);
	}


	@Override
	public void initialize() {
		coveredTimer = 0;
		ElevatorSubsystem.ELEVATOR_POSITION.HPI.goTo();
		endEffectorSubsystem.setVelocity(1000.0);
	}

	@Override
	public void execute() {
		coveredTimer = Constants.backSensor() ? coveredTimer + 1 : 0;
	}

	@Override
	public boolean isFinished() {
		return coveredTimer > 5 || Constants.frontSensor();
	}

	@Override
	public void end(boolean interrupted) {
		endEffectorSubsystem.stopAll();
		//ElevatorSubsystem.ELEVATOR_POSITION.SAFE.goTo();
	}
}
