package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;


public class LeftStationFaceCommand extends A05DriveCommand {
    public LeftStationFaceCommand() {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance());
    }

    @Override
    public void execute() {
        conditionStick();
        navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, 54.0)));
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationKp();
        conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return !Utl.inTolerance(driveXbox.getRightX(), 0.0, 0.05)
                || RobotContainer.altLeftBumper.getAsBoolean()
                || Constants.getDPad(Constants.DRIVE_XBOX) != A05Constants.D_PAD.NONE;
    }
}
