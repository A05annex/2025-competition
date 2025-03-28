package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05Constants.D_PAD;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.commands.A05TagTargetCommand;

/**
 * This command is meant to act as an intermediary between the rest of the code and the TagTargetCommand when scoring on
 * the reef. Because seeing more than one tag around the reef is possible, the code first determines which tag the robot
 * is laterally closest to, then schedules a TagTargetCommand to that tag. The Y position is set dynamically based on
 * the D Pad where no input,up, or down will place the robot on center and any left or right input will place the robot
 * in front of the respective coral post.
 */
@SuppressWarnings("unused")
public class ReefTargetCommand extends Command {
    private final double xPosition;

	private D_PAD direction = null;

	private A05TagTargetCommand tagTargetCommand;

	private int wasScheduled = 0;

    private final String[] keyList = {"close center reef", "far center reef", "close left reef", "far left reef", "close right reef", "far right reef"};

	private String scheduledTagSet;

    public ReefTargetCommand(double xPosition) {
        this.xPosition = xPosition;
    }

	public ReefTargetCommand(double xPosition, D_PAD direction) {
		this.xPosition = xPosition;
		this.direction = direction;
	}

    @Override
    public void initialize() {
        super.initialize();

		wasScheduled = 0;
		tagTargetCommand = null;
		direction = null;
		scheduledTagSet = "";
    }

    @Override
    public void execute() {
		if(tagTargetCommand != null) {
			return;
		}

		double bestAngle = 360.0;
		String bestTagSetKey = "";

		// Iterate through the tag sets and find the one with the lowest angle
		for(String key : keyList) {
			InferredRobotPosition robotPosition = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(key)); // Get the position

			double newAngle = Math.abs(Math.atan(robotPosition.y / robotPosition.x));

			// The position needs to be new (the robot just saw the tag), it cannot be the lastKey (already checked).
			// If the new angle is lower than the current best (lowest) angle by 10 degrees, update the lowest angle and best tag set key
			if(robotPosition.isNew && !key.equals(bestTagSetKey) && newAngle < bestAngle - (10.0 * Math.PI / 180.0)) {
				bestAngle = newAngle;
				bestTagSetKey = key;
			}
		}

		if(bestAngle == 360.0 || bestTagSetKey.isEmpty()) {
			return;
		}

		double cameraOffset = 0.07;
		double coralSpacing = 0.1951;
		direction = direction == null ? Constants.getDPad(A05Constants.ALT_XBOX) : direction;
		double yPosition = switch (direction) {
			case L, UL, DL -> cameraOffset + coralSpacing;
			case R, UR, DR -> cameraOffset - coralSpacing;
			default -> cameraOffset;
		};
		SmartDashboard.putNumber("YPos", yPosition);

		tagTargetCommand = new TagTargetCommand(xPosition, yPosition, bestTagSetKey);
		wasScheduled = 1;
		tagTargetCommand.schedule();
		scheduledTagSet = bestTagSetKey;
    }

    @Override
    public boolean isFinished() {
		wasScheduled = wasScheduled != 0 ? wasScheduled + 1 : 0;
        return tagTargetCommand != null && !tagTargetCommand.isScheduled() && wasScheduled >= 15;
    }

    @Override
    public void end(boolean interrupted) {
		if(tagTargetCommand != null) {
			tagTargetCommand.cancel();
		}

		SmartDashboard.putNumber("YPos", -1000);
    }

	public String getScheduledTagSet() {
		return scheduledTagSet;
	}
}
