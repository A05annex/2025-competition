package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class L1ScoreCommandGroup extends SequentialCommandGroup {
    public L1ScoreCommandGroup() {
        super(new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.L1), new TagTargetCommand(0.5,0.0, "reef close"), new CoralTroughScoreCommand(true));
    }
}