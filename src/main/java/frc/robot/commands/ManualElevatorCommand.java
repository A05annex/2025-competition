package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import org.a05annex.util.Utl;


public class ManualElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    private boolean wasSpinning;

    public ManualElevatorCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        wasSpinning = false;
    }

    @Override
    public void execute() {
        if (!Utl.inTolerance(Constants.ALT_XBOX.getRightY(), 0.0, 0.05)) {
            elevatorSubsystem.goToDeltaPosition(-8.0 * Constants.ALT_XBOX.getRightY());
            wasSpinning = true;
            return;
        }

        if (wasSpinning) {
            wasSpinning = false;
            elevatorSubsystem.goToDeltaPosition(0.0);
        }
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
