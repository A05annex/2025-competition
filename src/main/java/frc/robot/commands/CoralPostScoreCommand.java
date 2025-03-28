package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class CoralPostScoreCommand extends Command {
    //private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();

    private int timeSpun;

    private int raceTimer;

    private boolean spinning = false;

    public CoralPostScoreCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.endEffectorSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        timeSpun = 0;
        raceTimer = 0;
        spinning = false;
        if(ElevatorSubsystem.getInstance().isInPosition()) {
            endEffectorSubsystem.setVelocity(2000.0);
            System.out.println("spinning *************************************");
            spinning = true;
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()} returns true.)
     */
    @Override
    public void execute() {
        raceTimer++;
        if(!spinning && (ElevatorSubsystem.getInstance().isInPosition() || raceTimer > 175)) {
            endEffectorSubsystem.setVelocity(2000.0);
            System.out.println("spinning *************************************");
            spinning = true;
        }
        timeSpun = !Constants.frontSensor() ? timeSpun + 1 : 0;
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return timeSpun > 10 || raceTimer > 250;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is, it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.stopAll();
    }
}
