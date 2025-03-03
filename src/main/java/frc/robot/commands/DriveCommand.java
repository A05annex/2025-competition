package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
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

        conditionStick();


        for(A05Constants.D_PAD direction : A05Constants.D_PAD.values()) {

            if(Constants.getDPad(driveXbox) == direction || (RobotContainer.altA.getAsBoolean() && A05Constants.getDPad(Constants.ALT_XBOX) == direction)) {
                lastDirection = direction;
                isTargetingHeading = direction != A05Constants.D_PAD.NONE;
            }
        }

        if(!Utl.inTolerance(driveXbox.getRightX(), 0.0, 0.05)) {
            isTargetingHeading = false;
        }

        if(isTargetingHeading) {
            calcTargetHeadingRotation();
        }

		System.out.println(isTargetingHeading);

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
            case R -> 90.0;
            case L -> 270.0;
			default -> -1.0;
		};

		 if(heading == -1.0) {
			 return;
		 }

        navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)));
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationKp();
        conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
	}
}