package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.a05annex.frc.A05Constants;


public class AlgaeDislodgeCommand extends Command {
    ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    AlgaeSubsystem algaeSubsystem = AlgaeSubsystem.getInstance();

    ElevatorSubsystem.ELEVATOR_POSITION position;

    public AlgaeDislodgeCommand() {
        addRequirements(elevatorSubsystem, algaeSubsystem);
    }

    @Override
    public void initialize() {
        position = Constants.getDPad(Constants.ALT_XBOX) == A05Constants.D_PAD.D ?
                ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW : ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_HIGH;
        elevatorSubsystem.goToMAXMotionPosition(position.position - elevatorSubsystem.inchesToEncoder(5.0));
        algaeSubsystem.setVelocity(3000.0);
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.isInPosition(position.position - elevatorSubsystem.inchesToEncoder(5.0))) {
            position.goTo();
        }
    }

    @Override
    public boolean isFinished() {
        return position.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();
        ElevatorSubsystem.ELEVATOR_POSITION.SAFE.goTo();
    }
}
