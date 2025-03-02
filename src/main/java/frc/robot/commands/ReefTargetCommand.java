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
    private double bestY = 1000.0;
    private String bestTagSetKey = null;

    private final double xPosition;

	private D_PAD direction = null;

	private A05TagTargetCommand tagTargetCommand;

	private int wasScheduled = 0;

    private final String[] keyList = {"close center reef", "far center reef", "close left reef", "far left reef", "close right reef", "far right reef"};

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
		bestY = 1000.0;
		bestTagSetKey = null;
		wasScheduled = 0;
		tagTargetCommand = null;
		direction = null;
    }

    @Override
    public void execute() {
		if(tagTargetCommand != null) {
			return;
		}

		for(String key : keyList) {
			InferredRobotPosition irp = InferredRobotPosition.getRobotPosition(Constants.aprilTagSetDictionary.get(key));
			if(irp.isValid) {
				bestY = Math.min(Math.abs(irp.y), bestY); // If the y value is less than the previous best y value, set the best y value to this y value
				bestTagSetKey = bestY == Math.abs(irp.y) ? key : bestTagSetKey; // If the current tag is now best, update the best tag set
			}
		}

		if(bestY == 1000.0) {
			return;
		}

		double cameraOffset = 0.08;
		double coralSpacing = 0.1651;
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
}
