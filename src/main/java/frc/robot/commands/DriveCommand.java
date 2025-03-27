package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.InferredRobotPosition;
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
	private final String[] keyList = {"close center reef", "far center reef", "close left reef", "far left reef", "close right reef", "far right reef"};
	private int driveTimeout;
	private int altTimeout;
	private String lastKey = "";
	private HeadingSource headingSource = HeadingSource.NONE;

	private enum HeadingSource {
		NONE,
		XBOX,
		APRILTAG,
		LEFT_STATION,
		RIGHT_STATION
	}

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
		resetState();
	}

	@Override
	public void execute() {
		double rotate = lastConditionedRotate;

		conditionStick(); // Do the normal drive selection based on the drive sticks

		chooseHeadingSource(); // Check if we have inputs which would use an alternate rotation algorithm

		System.out.println();

		// If the right stick is not centered (driver is trying to spin), reset the state
		if(!Utl.inTolerance(driveXbox.getRightX(), 0.0, 0.05)) {
			resetState();
		}

		// If we have a heading source, calculate the rotation
		if(headingSource != HeadingSource.NONE) {
			lastConditionedRotate = rotate;
			switch (headingSource) {
				case XBOX -> calcXboxHeadingRotation();
				case APRILTAG -> calcAprilTagHeadingRotation();
				case LEFT_STATION -> targetHeading(54.0);
				case RIGHT_STATION -> targetHeading(-54.0);
			}
		}
		iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
	}

	private void resetState() {
		direction = A05Constants.D_PAD.NONE;
		altTimeout = 0;
		driveTimeout = 0;
		lastKey = "";
		headingSource = HeadingSource.NONE;
	}

	private void chooseHeadingSource() {
		if(RobotContainer.driveStart.getAsBoolean()) {
			headingSource = HeadingSource.APRILTAG;
			return;
		}

		if(RobotContainer.driveLeftBumper.getAsBoolean()) {
			headingSource = HeadingSource.LEFT_STATION;
			return;
		}

		if(RobotContainer.driveRightBumper.getAsBoolean()) {
			headingSource = HeadingSource.RIGHT_STATION;
			return;
		}

		for(A05Constants.D_PAD direction : A05Constants.D_PAD.values()) {
			updateTimeouts(direction);
			if(direction == A05Constants.D_PAD.NONE) {
				continue;
			}
			if(Constants.getDPad(driveXbox) == direction && driveTimeout <= 0) {
				driveTimeout = this.direction != A05Constants.D_PAD.NONE ? 3 : 0;
				this.direction = direction;
				headingSource = HeadingSource.XBOX;
			} else if(RobotContainer.altLeftBumper.getAsBoolean() && Constants.getDPad(Constants.ALT_XBOX) == direction && altTimeout <= 0) {
				altTimeout = this.direction != A05Constants.D_PAD.NONE ? altTimeout - 1 : 0;
				this.direction = direction;
				headingSource = HeadingSource.XBOX;
			}
		}
	}

	private void updateTimeouts(A05Constants.D_PAD direction) {
		if((driveTimeout > 0 && Constants.getDPad(driveXbox) == direction)
				|| (altTimeout > 0 && RobotContainer.altLeftBumper.getAsBoolean() && Constants.getDPad(Constants.ALT_XBOX) == direction)
				&& direction != this.direction) {
			driveTimeout = driveTimeout > 0 ? driveTimeout - 1 : 0;
			altTimeout = altTimeout > 0 ? altTimeout - 1 : 0;
		}
	}

	private void calcXboxHeadingRotation() {
		double heading =
				switch(direction) {
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

		targetHeading(heading);
	}

	private void calcAprilTagHeadingRotation() {
		double bestAngle = 360.0;
		String bestTagSetKey = "";


		// If the lastKey is set (not empty), set lowestAngle to its value (if possible). This will bias the algorithm
		// towards the last tag set to remove oscillation
		if(!lastKey.isEmpty()) {
			InferredRobotPosition lastRobotPosition = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(lastKey));
			if(lastRobotPosition.isValid) {
				bestAngle = Math.abs(Math.atan(lastRobotPosition.y / lastRobotPosition.x));
			}
		}

		// Iterate through the tag sets and find the one with the lowest angle
		for(String key : keyList) {
			InferredRobotPosition robotPosition = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(key)); // Get the position

			double newAngle = Math.abs(Math.atan(robotPosition.y / robotPosition.x));

			// The position needs to be new (the robot just saw the tag), it cannot be the lastKey (already checked).
			// If the new angle is lower than the current best (lowest) angle by 10 degrees, update the lowest angle and best tag set key
			if(robotPosition.isNew && !key.equals(lastKey) && newAngle < bestAngle - (10.0 * Math.PI / 180.0)) {
				bestAngle = newAngle;
				bestTagSetKey = key;
				lastKey = key;
			}
		}

		bestTagSetKey = bestTagSetKey.isEmpty() ? lastKey : bestTagSetKey;

		if(bestAngle == 360.0 || bestTagSetKey.isEmpty()) {
			lastKey = "";
			return;
		}

		targetHeading(Constants.aprilTagSetDictionary.get(bestTagSetKey).heading().getDegrees());
	}

	private void targetHeading(double heading) {
		navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)));
		conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
				.getRadians() * A05Constants.getDriveOrientationKp();
		conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
	}
}
