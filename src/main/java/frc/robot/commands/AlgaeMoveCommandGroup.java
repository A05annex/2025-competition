package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import org.a05annex.frc.A05Constants;

public class AlgaeMoveCommandGroup extends SequentialCommandGroup {
    public AlgaeMoveCommandGroup() {
        super(new ConditionalCommand(new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_HIGH::goTo), new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW::goTo), () -> A05Constants.getDPad(A05Constants.ALT_XBOX) == A05Constants.D_PAD.U));
    }
}