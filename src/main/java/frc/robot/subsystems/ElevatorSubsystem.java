package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotStateManager;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.ELEVATOR_MOTOR);

    // Declare min and max soft limits and where the motor thinks it starts
    @SuppressWarnings("FieldCanBeLocal")
    private final Double minPosition = -1.0, maxPosition = 96.5;

    @SuppressWarnings("FieldCanBeLocal")
	private final double
    // Declare PID constants for smart motion control
            mmKp = 0.3, mmKi = 0.0, mmKiZone = 0.0, mmKd = 0.0, mmMaxRPM = 10000.0, mmMaxDeltaRPMSec = 10000.0, mmError = 0.5,

    // Declare PID constants for position control
            posKp = 0.3, posKi = 0.002, posKiZone = 1.0, posKff = 0.0,

    // Declare PID constants for speed (rpm) control
            rpmKp = 0.1, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.000156,

            coralCollisionMinHeight = 27.0, coralCollisionMaxHeight = 63.0,

            positionTolerance = 0.5,

            analogEncoderZero = 0.042,

            gearRatio = 27.0,

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
        motor.setIdleMode(SparkBaseConfig.IdleMode.kCoast);
        motor.setPositionPID(posKp, posKi, posKiZone, posKff);
        motor.setMAXMotionPosition(mmKp, mmKi, mmKiZone, mmKd, mmMaxRPM, mmMaxDeltaRPMSec, mmError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(encoderStartPosition());
        requestedPosition = encoderStartPosition();
    }

    public boolean goToMAXMotionPosition(double position) {
        if(RobotContainer.altStart.getAsBoolean()) {
            motor.setTargetMAXMotionPosition(position);
            requestedPosition = position;
            return true;
        }

        if(!RobotStateManager.CoralManager.elevatorBlocked()) {
            System.out.println("goToMAXMotionPosition: " +  position);
            motor.setTargetPosition(position);
            requestedPosition = position;
            return true;
        }

        if((getPosition() < coralCollisionMinHeight && position <= coralCollisionMinHeight) //Not crossing from above or below the coral collision zone
                || (getPosition() >= coralCollisionMaxHeight && position > coralCollisionMaxHeight)) {

            System.out.println("goToMAXMotionPosition: " +  position);
            motor.setTargetPosition(position);
            requestedPosition = position;
            return true;
        }
        System.out.println("goToMAXMotionPosition: rejected");
        return false;
    }

    public void goToDeltaPosition(double delta){
        goToMAXMotionPosition(getPosition() + delta);
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
        double shift = -0.12;
        double encoder = Constants.ELEVATOR_ANALOG_ENCODER.get() + shift;
        return encoder < 0.0 ? encoder + 1.0 : encoder;
    }

    private double encoderStartPosition() {
        return (getCorrectedEncoder() - analogEncoderZero) * gearRatio + 0.0;
    }

    public enum ELEVATOR_POSITION {
        AGI(11.0 * 27.0 / 45.0),
        HPI(30.0),
        SAFE(5.0),
        ALGAE_HIGH(62.5),
        ALGAE_LOW(25.7),
        ALGAE_HOLD(90.0 * 27.0 / 45.0),
        L1(33.1),
        L2(57.0),
        L3(95.8);

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
}

