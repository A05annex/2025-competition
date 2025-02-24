package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateManager;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.ELEVATOR_MOTOR);

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = 0.0, maxPosition = 171.5;

    @SuppressWarnings("FieldCanBeLocal")
	private final double
    // Declare PID constants for smart motion control
            mmKp = 0.3, mmKi = 0.0, mmKiZone = 0.0, mmKd = 0.0, mmMaxRPM = 5200.0, mmMaxDeltaRPMSec = 10000.0, mmError = 0.2,

    // Declare PID constants for position control
            posKp = 0.1, posKi = 0.0, posKiZone = 0.0, posKff = 0.0,

    // Declare PID constants for speed (rpm) control
            rpmKp = 0.0001, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.000156,

            AGICollisionHeight = 28.0, coralCollisionMinHeight = 60.0, coralCollisionMaxHeight = 107.0,

            positionTolerance = 0.3,

            analogEncoderZero = 0.0123,

            gearRatio = 45.0,

            encoderToInches = 42.0 / (maxPosition - minPosition);

    private double requestedPosition;

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
        motor.setMAXMotionPosition(mmKp, mmKi, mmKiZone, mmKd, mmMaxRPM, mmMaxDeltaRPMSec, mmError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(encoderStartPosition());
        requestedPosition = encoderStartPosition();
    }

    public boolean goToMAXMotionPosition(double position) {
        if(position >= AGICollisionHeight && !RobotStateManager.CoralManager.elevatorBlocked()) {
            RobotStateManager.ElevatorAGIManager.releaseAccess(this);
            motor.setTargetMAXMotionPosition(position);
            requestedPosition = position;
            return true;
        }

        if(position < AGICollisionHeight) {
            if(RobotStateManager.ElevatorAGIManager.requestAccess(this)) {
                motor.setTargetMAXMotionPosition(position);
                requestedPosition = position;
                return true;
            }
            else {
                return false;
            }
        }

        if(!((getPosition() < coralCollisionMinHeight && position >= coralCollisionMinHeight) //Not crossing from above or below the coral collision zone
                || (getPosition() >= coralCollisionMaxHeight && position < coralCollisionMaxHeight))) {
                motor.setTargetMAXMotionPosition(position);
                requestedPosition = position;
                return true;
        }
        return false;
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getPosition() {
        return motor.getEncoderPosition();
    }

    public boolean isInPosition(double position) {
        return Utl.inTolerance(getPosition(), position, positionTolerance);
    }

    public boolean isInPosition() {
        return Utl.inTolerance(getPosition(), requestedPosition, positionTolerance);
    }

    public double getCorrectedEncoder() {
        double shift = -0.13;
        double encoder = Constants.ELEVATOR_ANALOG_ENCODER.get() + shift;
        return encoder < 0.0 ? encoder + 1.0 : encoder;
    }

    private double encoderStartPosition() {
        return (getCorrectedEncoder() - analogEncoderZero) * gearRatio;
    }

    public enum ELEVATOR_POSITION {
        AGI(11.0),
        HPI(66.25),
        SAFE(35.0),
        ALGAE_HIGH(167.0),
        ALGAE_LOW(120.0),
        ALGAE_HOLD(90.0),
        L1(70.8),
        L2(110.0),
        L3(171.3);

        public final double position;

        private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

        ELEVATOR_POSITION(double position) {
            this.position = position;
        }

        public boolean goTo() {
            return elevatorSubsystem.goToMAXMotionPosition(position);
        }

        public boolean isInPosition() {
            return elevatorSubsystem.isInPosition(position);
        }
    }

    public double encoderToInches(double encoder) {
        return encoder * encoderToInches;
    }

    public double inchesToEncoder(double inches) {
        return inches / encoderToInches;
    }
}

