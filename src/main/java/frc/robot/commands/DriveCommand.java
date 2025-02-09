package frc.robot.commands;

import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {
    private A05Constants.D_PAD lastDirection = A05Constants.D_PAD.NONE;

    private boolean isTargetingHeading = false;

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
        lastDirection = A05Constants.D_PAD.NONE;
        isTargetingHeading = false;
    }

    @Override
    public void execute() {
        //This runs the default swerve calculations for xbox control
        //super.execute();

        for(A05Constants.D_PAD direction : A05Constants.D_PAD.values()) {
            if(Constants.getDPad(A05Constants.DRIVE_XBOX) == direction) {
                lastDirection = direction;
                isTargetingHeading = true;
            }
        }

        if(Utl.inTolerance(Constants.DRIVE_XBOX.getRightX(), 0.0, driver.getRotateDeadband())) {
            isTargetingHeading = false;
        }

        if(isTargetingHeading) {
            calcTargetHeadingRotation();
        }

        conditionStick();

        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    private void calcTargetHeadingRotation() {
		 double heading = switch(lastDirection) {
			case U -> 0.0;
			case UR -> 60.0;
			case DR -> 120.0;
			case D -> 180.0;
			case DL -> 240.0;
			case UL -> 300.0;
			default -> 0.0;
		};

        navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)));
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationKp();
        conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
	}
}