package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class L1ScoreCommandGroup extends SequentialCommandGroup {
    public L1ScoreCommandGroup() {
        super(Commands.parallel(new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.L1), new ReefTargetCommand(0.5)), new CoralTroughScoreCommand(true));
    }
}