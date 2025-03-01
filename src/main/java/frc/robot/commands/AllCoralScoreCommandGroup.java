package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.ICanTakeDrive;

public class AllCoralScoreCommandGroup extends SequentialCommandGroup implements ICanTakeDrive {
	private static A05Constants.D_PAD direction;

	public AllCoralScoreCommandGroup() {
		super(new ConditionalCommand(
				new L3ScoreCommandGroup(),
				new ConditionalCommand(
						new L2ScoreCommandGroup(),
						new L1ScoreCommandGroup(),
						AllCoralScoreCommandGroup::runL2),
				AllCoralScoreCommandGroup::runL3));
	}

	public AllCoralScoreCommandGroup(A05Constants.D_PAD direction) {
		super(new ConditionalCommand(
				new L3ScoreCommandGroup(),
				new ConditionalCommand(
						new L2ScoreCommandGroup(),
						new L1ScoreCommandGroup(),
						() -> runL2(direction)
				), () ->
				runL3(direction)
		));
	}

    public static boolean runL3() {
		direction = A05Constants.getDPad(Constants.ALT_XBOX);
        return direction == A05Constants.D_PAD.UL || direction == A05Constants.D_PAD.UR;
    }

	public static boolean runL3(A05Constants.D_PAD direction) {
		return direction == A05Constants.D_PAD.UL || direction == A05Constants.D_PAD.UR;
	}


	public static boolean runL2() {
		direction = A05Constants.getDPad(Constants.ALT_XBOX);
        return direction == A05Constants.D_PAD.L || direction == A05Constants.D_PAD.R;
    }

	public static boolean runL2(A05Constants.D_PAD direction) {
		return direction == A05Constants.D_PAD.L || direction == A05Constants.D_PAD.R;
	}

	@Override
	public boolean canTakeDrive() {
		return true;
	}
}