package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class L3ScoreCommandGroup extends SequentialCommandGroup {
    public L3ScoreCommandGroup() {
        super(new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.L3), new TagTargetCommand(0.5,0.0, "reef close"), new CoralPostScoreCommand());
    }
}