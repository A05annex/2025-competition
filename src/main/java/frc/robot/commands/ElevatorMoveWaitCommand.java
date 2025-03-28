package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorMoveWaitCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    private final double position;

    private boolean moveSuccessful = false;

    private int raceTimer;

    public ElevatorMoveWaitCommand(ElevatorSubsystem.ELEVATOR_POSITION position) {
       this.position = position.position;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @SuppressWarnings("unused")
	public ElevatorMoveWaitCommand(Double position) {
        this.position = position;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
    }

    @SuppressWarnings("unused")
	public ElevatorMoveWaitCommand(String positionName) {
        addRequirements(this.elevatorSubsystem);

        for(ElevatorSubsystem.ELEVATOR_POSITION pos : ElevatorSubsystem.ELEVATOR_POSITION.values()) {
            if(positionName.equals(pos.name())) {
                this.position = pos.position;
                return;
            }
        }
        position = -1.0;

        throw new IllegalArgumentException(String.format("ElevatorMoveWaitCommand(String positionName) was called " +
                "with an invalid name: %s. Ensure your String matches the name of a position in ElevatorSubsystem.ELEVATOR_POSITION", positionName));
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        raceTimer = 0;
        moveSuccessful = elevatorSubsystem.goToMAXMotionPosition(position);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()} returns true.)
     */
    @Override
    public void execute() {
        raceTimer++;
        if(!moveSuccessful) {
            moveSuccessful = elevatorSubsystem.goToMAXMotionPosition(position);
        }
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
        return elevatorSubsystem.isInPosition(position) || raceTimer > 250;
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
    }
}
