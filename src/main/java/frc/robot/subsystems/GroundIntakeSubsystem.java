package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class GroundIntakeSubsystem extends SubsystemBase {

    private final SparkNeo intakeMotor = SparkNeo.factory(Constants.CAN_Devices.GROUND_INTAKE_SPIN_MOTOR);

    private final SparkNeo actuatorMotor = SparkNeo.factory(Constants.CAN_Devices.GROUND_INTAKE_ACTUATOR_MOTOR);

    // Declare PID constants for smart motion control
    @SuppressWarnings("FieldCanBeLocal")
	private final double actuatorMmKp = 0.00005, actuatorMmKi = 0.000, actuatorMmKiZone = 0.0, actuatorMmKff = 0.000156, actuatorMmMaxRPM = 5000.0,
            actuatorMmMaxDeltaRPMSec = 3000.0, actuatorMmError = 0.1;

    // Declare PID constants for position control
    @SuppressWarnings("FieldCanBeLocal")
    private final double actuatorPosKp = 0.22, actuatorPosKi = 0.0, actuatorPosKiZone = 0.0, actuatorPosKff = 0.0;

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double intakeRpmKp = 0.5, intakeRpmKi = 0.0, intakeRpmKiZone = 0.0, intakeRpmKff = 0.0;

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double intakeMinPosition = 0.0, intakeMaxPosition = 1000.0, intakeStartPosition = 0.0;

    private final Double actuatorMinPosition = 0.0, actuatorMaxPosition = 1000.0, actuatorStartPosition = 0.0;

    // Declare non-parameter method constants
    private final double intakeRPM = 1000.0, extendedPosition = 1000.0, retractedPosition = 0.0;

    private final static GroundIntakeSubsystem INSTANCE = new GroundIntakeSubsystem();
    public static GroundIntakeSubsystem getInstance() {
        return INSTANCE;
    }

    private GroundIntakeSubsystem() {
        intakeMotor.startConfig();
        intakeMotor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps30);
		//noinspection ConstantValue
		intakeMotor.setSoftLimits(intakeMinPosition, intakeMaxPosition);
        intakeMotor.setDirection(SparkNeo.Direction.DEFAULT);
        //motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeMotor.setRpmPID(intakeRpmKp, intakeRpmKi, intakeRpmKiZone, intakeRpmKff);
        intakeMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeMotor.endConfig();
        intakeMotor.setEncoderPosition(intakeStartPosition);

        actuatorMotor.startConfig();
        actuatorMotor.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        //noinspection ConstantValue
        actuatorMotor.setSoftLimits(actuatorMinPosition, actuatorMaxPosition);
        actuatorMotor.setDirection(SparkNeo.Direction.DEFAULT);
        //motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        actuatorMotor.setPositionPID(actuatorPosKp, actuatorPosKi, actuatorPosKiZone, actuatorPosKff);
        actuatorMotor.setMAXMotionPosition(actuatorMmKp, actuatorMmKi, actuatorMmKiZone, actuatorMmKff, actuatorMmMaxRPM, actuatorMmMaxDeltaRPMSec, actuatorMmError);
        actuatorMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        actuatorMotor.endConfig();
        actuatorMotor.setEncoderPosition(actuatorStartPosition);
    }
    public void intake(){
        setIntakeVelocity(intakeRPM);
    }

    @SuppressWarnings("unused")
    public void setIntakeVelocity(double rpm) {
        intakeMotor.setTargetRPM(rpm);
    }

    @SuppressWarnings("unused")
    public void stopSpin() {
        intakeMotor.stopMotor();
    }


    public void actuatorToMMPosition(double position){
        actuatorMotor.setTargetMAXMotionPosition(position);
    }

    //TODO: determine if this is needed
    public void actuatorToPosition(double position) {
        actuatorMotor.setTargetPosition(position);
    }

    @SuppressWarnings("unused")
    public void stopActuator() {
        actuatorMotor.stopMotor();
    }

    public void extendActuator(){
        actuatorToMMPosition(extendedPosition);
    }

    public void retractActuator(){
        actuatorToMMPosition(retractedPosition);
    }

    public void stopAll(){
        stopSpin();
        stopActuator();
    }

    public double getIntakeVelocity(){
        return intakeMotor.getEncoderVelocity();
    }

    public double getActuatorPosition() {
        return actuatorMotor.getEncoderPosition();
    }
}
