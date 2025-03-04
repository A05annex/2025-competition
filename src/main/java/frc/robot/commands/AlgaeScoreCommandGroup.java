package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeScoreCommandGroup extends SequentialCommandGroup {
    public AlgaeScoreCommandGroup() {
        super(Commands.parallel(new ReefTargetCommand(-0.5), new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW.position - 10)), new AlgaeDislodgeCommand());
    }
}