package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.frc.subsystems.SparkNeo;

public class SampleMotorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(-1);

    // Declare PID constants for smart motion control
    @SuppressWarnings("FieldCanBeLocal")
	private final double mmKp = 0.00005, mmKi = 0.000, mmKiZone = 0.0, mmKd = 0.0, mmMaxRPM = 3000.0,
            mmMaxDeltaRPMSec = 3000.0, mmError = 0.1;

    // Declare PID constants for position control
    @SuppressWarnings("FieldCanBeLocal")
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double rpmKp = 0.5, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = null, maxPosition = 1000.0, startPosition = 500.0;

    private final static SampleMotorSubsystem INSTANCE = new SampleMotorSubsystem();
    public static SampleMotorSubsystem getInstance() {
        return INSTANCE;
    }

    private SampleMotorSubsystem() {
        motor.startConfig();
        motor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
		//noinspection ConstantValue
		motor.setSoftLimits(minPosition, maxPosition);
        motor.setDirection(SparkNeo.Direction.REVERSE);
        //motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motor.setPositionPID(posKp, posKi, posKiZone, posKff);
        motor.setMAXMotionPosition(mmKp, mmKi, mmKiZone, mmKd, mmMaxRPM, mmMaxDeltaRPMSec, mmError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(startPosition);
    }

    public void setVelocity(double rpm) {
        motor.setTargetRPM(rpm);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getPosition() {
        return motor.getEncoderPosition();
    }
}

