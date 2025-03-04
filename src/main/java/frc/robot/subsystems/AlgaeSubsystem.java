package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.ALGAE_MOTOR);

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double rpmKp = 0.00016, rpmKi = 0.0000015, rpmKiZone = 150.0, rpmKff = 0.000190;

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = null, maxPosition = null, startPosition = 0.0;

    private final static AlgaeSubsystem INSTANCE = new AlgaeSubsystem();
    public static AlgaeSubsystem getInstance() {
        return INSTANCE;
    }

    private AlgaeSubsystem() {
        motor.startConfig();
        motor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
		//noinspection ConstantValue
		motor.setSoftLimits(minPosition, maxPosition);
        motor.setDirection(SparkNeo.Direction.REVERSE);
        //motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(startPosition);
    }

    public void setVelocity(double rpm) {
        motor.setTargetRPM(rpm);
    }

    public void spin() {
        setVelocity(2000.0);
        //motor.sparkMax.set(0.5);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getVelocity() {
        return motor.getEncoderVelocity();
    }
}

