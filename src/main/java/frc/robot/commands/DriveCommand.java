package frc.robot.commands;

import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.ISwerveDrive;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {

    /**
     * Default command for DriveSubsystem. Left stick moves the robot field-relatively, and right stick X rotates.
     * Contains driver constants for sensitivity, gain, and deadband.
     */
    public DriveCommand(ISwerveDrive swerveDrive) {
        super(swerveDrive);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //This runs the default swerve calculations for xbox control
        super.execute();

        //conditionStick();
        //iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }
}