package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.a05annex.frc.A05Constants;


public class AlgaeMoveCommand extends Command {
	private final AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();
	private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

	private double position = 0.0;

	private final ReefTargetCommand reefTargetCommand;

	@SuppressWarnings("unused")
	public AlgaeMoveCommand() {
		addRequirements(this.algaeSubsystem, this.elevatorSubsystem);

		reefTargetCommand = null;
	}

	public AlgaeMoveCommand(ReefTargetCommand reefTargetCommand) {
		addRequirements(this.algaeSubsystem, this.elevatorSubsystem);

		this.reefTargetCommand = reefTargetCommand;
	}

	@Override
	public void initialize() {
		algaeSubsystem.spin();

		if(reefTargetCommand != null) {
			setPositionFromTargetCommand();
		} else {
			position = A05Constants.getDPad(A05Constants.ALT_XBOX) == A05Constants.D_PAD.U
					? ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_HIGH.position
					: ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW.position;
		}
	}

	@Override
	public void execute() {
		if(position == -1.0 && setPositionFromTargetCommand()) {
			elevatorSubsystem.goToMAXMotionPosition(position);
		}
	}

	@SuppressWarnings("RedundantMethodOverride")
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		algaeSubsystem.stop();
	}



	private boolean setPositionFromTargetCommand() {
		if(reefTargetCommand.getScheduledTagSet() == null || reefTargetCommand.getScheduledTagSet().isEmpty()) {
			position = -1.0;
			return false;
		}

		final int tagID = Constants.aprilTagSetDictionary.get(reefTargetCommand.getScheduledTagSet()).tagIDs()[0];

		final boolean high = (tagID % 2 != 0) == Constants.isRedAlliance();

		position = high
				? ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_HIGH.position
				: ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW.position;
		return true;
	}
}
