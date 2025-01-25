package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class EndEffectorSubsystem extends SubsystemBase {

    private final SparkNeo motorLeft = SparkNeo.factory(Constants.CAN_Devices.SAMPLE_MOTOR);

    private final SparkNeo motorRight = SparkNeo.factory(Constants.CAN_Devices.SAMPLE_MOTOR);

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double rpmKp = 0.001, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = null, maxPosition = null, startPosition = 0.0;

    private final static EndEffectorSubsystem INSTANCE = new EndEffectorSubsystem();
    public static EndEffectorSubsystem getInstance() {
        return INSTANCE;
    }

    private EndEffectorSubsystem() {
        motorLeft.startConfig();
        motorLeft.setCurrentLimit(SparkNeo.UseType.RPM_PROLONGED_STALL, SparkNeo.BreakerAmps.Amps30);
		//noinspection ConstantValue
		motorLeft.setSoftLimits(minPosition, maxPosition);
        motorLeft.setDirection(SparkNeo.Direction.REVERSE);
        motorLeft.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motorLeft.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motorLeft.endConfig();
        motorLeft.setEncoderPosition(startPosition);

        motorRight.startConfig();
        motorRight.setCurrentLimit(SparkNeo.UseType.RPM_PROLONGED_STALL, SparkNeo.BreakerAmps.Amps30);
        //noinspection ConstantValue
        motorRight.setSoftLimits(minPosition, maxPosition);
        motorRight.setDirection(SparkNeo.Direction.REVERSE);
        motorRight.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motorRight.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motorRight.endConfig();
        motorRight.setEncoderPosition(startPosition);
    }


    public void setLeftVelocity(double rpm) {motorLeft.setTargetRPM(rpm);}

    public void stopLeft() {
        motorLeft.stopMotor();
    }

    public double getLeftVelocity() {
        return motorLeft.getEncoderVelocity();
    }

    public void setRightVelocity(double rpm) {motorRight.setTargetRPM(rpm);}

    public void stopRight() {
        motorRight.stopMotor();
    }

    public double getRightVelocity() {
        return motorRight.getEncoderVelocity();
    }

    public void setVelocity(double rpm) {
        setRightVelocity(rpm);
        setLeftVelocity(rpm);
    }

    public double getVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2;
    }

    public void stopAll() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }
}

