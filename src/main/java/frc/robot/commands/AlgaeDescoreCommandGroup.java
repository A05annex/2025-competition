package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AlgaeDescoreCommandGroup extends ParallelCommandGroup {

	public AlgaeDescoreCommandGroup() {
		ReefTargetCommand reefTargetCommand = new ReefTargetCommand(0.25);
		super.addCommands(reefTargetCommand, new AlgaeMoveCommand(reefTargetCommand));
	}
}