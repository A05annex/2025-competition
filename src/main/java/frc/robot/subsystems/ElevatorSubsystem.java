package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.ELEVATOR_MOTOR);

    // Declare PID constants for smart motion control
    @SuppressWarnings("FieldCanBeLocal")
	private final double mmKp = 0.3, mmKi = 0.0, mmKiZone = 0.0, mmKff = 0.000156, mmMaxRPM = 5200.0,
            mmMaxDeltaRPMSec = 10000.0, mmError = 0.4;

    // Declare PID constants for position control
    @SuppressWarnings("FieldCanBeLocal")
    private final double posKp = 0.1, posKi = 0.0, posKiZone = 0.0, posKff = 0.000156;

    // Declare PID constants for speed (rpm) control
    @SuppressWarnings("FieldCanBeLocal")
    private final double rpmKp = 0.0001, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.000156;

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = -0.1, maxPosition = 165.0, startPosition = 0.0;

    private final double positionTolerance = 0.3;

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private ElevatorSubsystem() {
        motor.startConfig();
        motor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
		motor.setSoftLimits(minPosition, maxPosition);
        motor.setDirection(SparkNeo.Direction.REVERSE);
        //motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        motor.setPositionPID(posKp, posKi, posKiZone, posKff);
        motor.setMAXMotionPosition(mmKp, mmKi, mmKiZone, mmKff, mmMaxRPM, mmMaxDeltaRPMSec, mmError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(startPosition);
    }

    @SuppressWarnings("unused")
    public void goToMAXMotionPosition(double position) {
        motor.setTargetMAXMotionPosition(position);
    }

    @SuppressWarnings("unused")
    public void resetEncoder() {
        motor.setEncoderPosition(0.0);
    }

    public void moveUp() {
        motor.sparkMax.set(1.0);
    }

    public void moveDown() {
        motor.sparkMax.set(-1.0);
    }

    @SuppressWarnings("unused")
    public void stop() {
        motor.stopMotor();
    }

    public double getPosition() {
        return motor.getEncoderPosition();
    }

    public boolean isInPosition(double position) {
        return Utl.inTolerance(getPosition(), position, positionTolerance);
    }

    public enum ELEVATOR_POSITION {
        AGI(0.0),
        HPI(20.0),
        L1(50.0),
        L2(100.0),
        L3(160.0);

        public final double position;

        private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

        ELEVATOR_POSITION(double position) {
            this.position = position;
        }

        public void goTo() {
            elevatorSubsystem.goToMAXMotionPosition(position);
            System.out.print("going to " + this.name());
        }

        public boolean isInPosition() {
            return elevatorSubsystem.isInPosition(position);
        }
    }
}

