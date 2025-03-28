package frc.robot.commands;

import org.a05annex.frc.commands.A05TagTargetCommand;
import org.a05annex.frc.commands.ICanTakeDrive;

@SuppressWarnings("unused")
public class TagTargetCommand extends A05TagTargetCommand implements ICanTakeDrive {

    public TagTargetCommand(Double xPosition, Double yPosition, String positionParametersKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(xPosition, yPosition, positionParametersKey);
    }

    @Override
    public void initialize() {
        super.initialize();
        MAX_SPEED = 1.0;
        POSITION_CONTROL_SPEED = 0.2;
        REDUCED_SPEED_RADIUS = 1.5;

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean canTakeDrive() {
        return super.canTakeDrive();
    }
}
