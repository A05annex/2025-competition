package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Constants;


public class CoralCenterCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();

    private STATUS currentStatus;

    @SuppressWarnings("FieldCanBeLocal")
    private final double slowSpeed = 200, fastSpeed = 600;

    private int centerTimer;

    public CoralCenterCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.endEffectorSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        currentStatus = null;
        centerTimer = 0;
        if (!Constants.backSensor() && !Constants.frontSensor()) {
            currentStatus = STATUS.STOPPED;
            endEffectorSubsystem.stopAll();
        } else if (!Constants.backSensor() && Constants.frontSensor()) {
            currentStatus = STATUS.FAST_BACK;
            endEffectorSubsystem.setVelocity(-fastSpeed);
        } else if ((Constants.backSensor() && !Constants.frontSensor()) || (Constants.backSensor() && Constants.frontSensor())) {
            currentStatus = STATUS.FAST_FORWARD;
            endEffectorSubsystem.setVelocity(fastSpeed);
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        switch (currentStatus) {
            case STOPPED:
                return;
            case SLOW_BACK:
                if (Constants.backSensor() && centerTimer >= 15) {
                    currentStatus = STATUS.STOPPED;
                    endEffectorSubsystem.stopAll();
                } else {
                    centerTimer = Constants.backSensor() ? centerTimer + 1 : 0;
                }
            case FAST_FORWARD, SLOW_FORWARD:
                if(!Constants.backSensor()) {
                    currentStatus = STATUS.SLOW_BACK;
                    endEffectorSubsystem.setVelocity(-slowSpeed);
                }
                break;
            case FAST_BACK:
                if (Constants.backSensor()) {
                    currentStatus = STATUS.SLOW_FORWARD;
                    endEffectorSubsystem.setVelocity(slowSpeed);
                }
                break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
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

    enum STATUS {
        FAST_FORWARD,
        FAST_BACK,
        SLOW_FORWARD,
        SLOW_BACK,
        STOPPED
    }
}
