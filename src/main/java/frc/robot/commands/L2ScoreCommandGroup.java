package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class L2ScoreCommandGroup extends SequentialCommandGroup {
    public L2ScoreCommandGroup() {
        super(new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.L2), new TagTargetCommand(0.5,0.0, "reef close"), new CoralPostScoreCommand());
    }
}