package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import org.a05annex.frc.A05Constants;

public class L1ScoreCommandGroup extends SequentialCommandGroup {
    public L1ScoreCommandGroup() {
        super(Commands.parallel(new ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION.L1), new ReefTargetCommand(-0.5, A05Constants.D_PAD.D)), new CoralTroughScoreCommand(Constants.getDPad(A05Constants.ALT_XBOX) == A05Constants.D_PAD.DR));
    }
}