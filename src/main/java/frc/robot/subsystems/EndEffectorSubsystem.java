package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class EndEffectorSubsystem extends SubsystemBase {

    private final SparkNeo motorLeft = SparkNeo.factory(Constants.CAN_Devices.END_EFFECTOR_LEFT_MOTOR);

    private final SparkNeo motorRight = SparkNeo.factory(Constants.CAN_Devices.END_EFFECTOR_RIGHT_MOTOR);

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double rpmKp = 0.00016, rpmKi = 0.0000015, rpmKiZone = 150.0, rpmKff = 0.000156;

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
        motorRight.setDirection(SparkNeo.Direction.DEFAULT);
        motorRight.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motorRight.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motorRight.endConfig();
        motorRight.setEncoderPosition(startPosition);
    }


    public void setLeftVelocity(double rpm) {motorLeft.setTargetRPM(rpm);}

    public void stopLeft() {
        motorLeft.stopMotor();
    }

    public void setRightVelocity(double rpm) {motorRight.setTargetRPM(rpm);}

    public void stopRight() {
        motorRight.stopMotor();
    }

    public void setVelocity(double rpm) {
        setRightVelocity(rpm);
        setLeftVelocity(rpm);
    }

    public void spin() {
        setVelocity(2000.0);
    }

    public void stopAll() {
        motorLeft.stopMotor();
        motorRight.stopMotor();
    }
}

