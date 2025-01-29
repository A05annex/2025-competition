package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffectorSubsystem;


public class CoralTroughScoreCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem = EndEffectorSubsystem.getInstance();

    private final boolean stopRight;
    private int timeSpun;

    public CoralTroughScoreCommand(boolean stopRight) {
       this.stopRight = stopRight;
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
        endEffectorSubsystem.setVelocity(2000.0);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
      if (!Constants.backSensor()){
		  if(stopRight) {
			  endEffectorSubsystem.stopRight();
		  } else {
			  endEffectorSubsystem.stopLeft();
		  }
      }


      timeSpun = Constants.frontSensor() ? timeSpun + 1 : 0;
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
        return timeSpun > 50;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
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
