package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.RobotPosition;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;

/**
 * Drive command is here because you will likely need to override the serve (targeting, competition specific reason)
 */
public class DriveCommand extends A05DriveCommand {
    private A05Constants.D_PAD direction = A05Constants.D_PAD.NONE;

    private boolean isTargetingHeading = false;

	private final String[] keyList = {"close center reef", "far center reef", "close left reef", "far left reef", "close right reef", "far right reef"};

	private int driveTimeout;
    private int altTimeout;

	private String lastKey = "";

	private int tagSwitchTimeout;

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
        direction = A05Constants.D_PAD.NONE;
        isTargetingHeading = false;
        altTimeout = 0;
		driveTimeout = 0;
		tagSwitchTimeout = 0;
		lastKey = "";
    }

    @Override
    public void execute() {
        //This runs the default swerve calculations for xbox control
        //super.execute();

        double rotate = lastConditionedRotate;

        conditionStick();


        for(A05Constants.D_PAD direction : A05Constants.D_PAD.values()) {
			if(RobotContainer.driveRightBumper.getAsBoolean()) {
				isTargetingHeading = true;
				this.direction = A05Constants.D_PAD.NONE;
				break;
			}

			if((driveTimeout > 0 && Constants.getDPad(driveXbox) == direction)
					|| (altTimeout > 0 &&  RobotContainer.altLeftBumper.getAsBoolean() && Constants.getDPad(Constants.ALT_XBOX) == direction)
					&& direction != this.direction) {
				driveTimeout = driveTimeout > 0 ? driveTimeout - 1 : 0;
				altTimeout = altTimeout > 0 ? altTimeout - 1 : 0;
			}

			if(direction == A05Constants.D_PAD.NONE) {
				continue;
			}

			if(Constants.getDPad(driveXbox) == direction && driveTimeout <= 0) {
				driveTimeout = this.direction != A05Constants.D_PAD.NONE ? 3 : 0;
				this.direction = direction;
				isTargetingHeading = true;
            } else if(RobotContainer.altLeftBumper.getAsBoolean() && Constants.getDPad(Constants.ALT_XBOX) == direction && altTimeout <= 0) {
				altTimeout = this.direction != A05Constants.D_PAD.NONE ? altTimeout - 1 : 0;
				this.direction = direction;
				isTargetingHeading = true;
            }
        }

        if(!Utl.inTolerance(driveXbox.getRightX(), 0.0, 0.05)) {
            isTargetingHeading = false;
			direction = A05Constants.D_PAD.NONE;

			altTimeout = 0;
			driveTimeout = 0;
			tagSwitchTimeout = 0;
        }

        if(isTargetingHeading) {
            lastConditionedRotate = rotate;
            calcTargetHeadingRotation();
        }

        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    private void calcTargetHeadingRotation() {
		double bestY = 1000.0;
		String bestTagSetKey = "";

		double heading = switch(direction) {
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

		if(!lastKey.isEmpty()) {
			InferredRobotPosition lastRobotPosition = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(lastKey));
			if(lastRobotPosition.isValid) {
				bestY = Math.abs(lastRobotPosition.y); // If the y value is less than the previous best y value, set the best y value to this y value
			}
		}

		if(direction == A05Constants.D_PAD.NONE) {
			for(String key : keyList) {
				InferredRobotPosition robotPosition = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(key));
				if(robotPosition.isNew && !key.equals(lastKey)) {
					System.out.println("New Reported Y: " + Math.abs(robotPosition.y) + "\nNew Key: " + key);
					if(Math.abs(robotPosition.y) < bestY - 0.8) {
						bestY = Math.abs(robotPosition.y);
						bestTagSetKey = key; // If the current tag is now best, update the best tag set
						lastKey = key;
					}
				}

				bestTagSetKey = bestTagSetKey.isEmpty() ? lastKey : bestTagSetKey;

				if(bestY == 1000.0) {
					lastKey = "";
					return;
				}
			}
			heading = Constants.aprilTagSetDictionary.get(bestTagSetKey).heading().getDegrees();
		} else {
			tagSwitchTimeout--;
		}

		 if(heading == -1.0) {
			 return;
		 }

		 if(tagSwitchTimeout <= 0 && !navX.getHeadingInfo().expectedHeading.equals(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)))) {
         	 navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)));
			 tagSwitchTimeout = 0;
		 } else {
			 tagSwitchTimeout--;
		 }

         conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationKp();
         conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
	}
}