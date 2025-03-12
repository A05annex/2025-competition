// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.photonvision.PhotonCamera;

import java.io.FileNotFoundException;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends A05Constants
{
    public static final class CAN_Devices {
        public static final int
            // Non-Drive Motors
            ALGAE_MOTOR = 12,
            ELEVATOR_MOTOR = 10,
            END_EFFECTOR_RIGHT_MOTOR = 13,
            END_EFFECTOR_LEFT_MOTOR = 14,
            GROUND_INTAKE_ACTUATOR_MOTOR = 11,
            GROUND_INTAKE_SPIN_MOTOR = 9;
    }

    @SuppressWarnings("unused")
    public static final boolean HAS_USB_CAMERA = false;
    @SuppressWarnings("unused")
	public static final boolean HAS_LIMELIGHT = false;

    public static boolean backSensor() {
        return !BACK_CORAL_SENSOR.get();
    }
    public static boolean frontSensor() {
        return !FRONT_CORAL_SENSOR.get();
    }

    public static final PhotonCameraWrapper CAMERA = new PhotonCameraWrapper(new PhotonCamera("Arducam_OV9281_USB_Camera"), Units.inchesToMeters(14.0), 0.0, 0.0);
//0.3429
    public static double xCorrectionFunction(double reportedX) {
        return 0.938148 * reportedX - 0.0116201;
    }

    public static double yCorrectionFunction(double reportedY) {
        return 1.01911 * reportedY - 0.0664051;
    }

    public static final DigitalInput FRONT_CORAL_SENSOR = new DigitalInput(7), BACK_CORAL_SENSOR = new DigitalInput(8);

    public static final DutyCycleEncoder ELEVATOR_ANALOG_ENCODER = new DutyCycleEncoder(9);

    // kP for keeping drive at the same orientation
    public static final double DRIVE_ORIENTATION_kP = 1.2;

    // for practice, length and width from center of the wheels, in m (note chassis is 30" square,
    // the bolt pattern is 29" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 23.5"(0.5969m) square.)
    // For competition, length and width from center of the wheels, in m (note chassis is 28" square,
    // the bolt pattern is 27" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 21.5"(0.5461m) square.)

    /**
     * The geometry and calibration specific to a swerve drive robot base. We currently have 2 bases, the first being
     * a <i>prototyping/practice</i> base that should always be in working condition for drive tuning/testing,
     * calibration, as well as software prototyping. The second is the <i>competition</i> robot that is built for
     * the yearly competition, and is generally not drivable between the introduction of the competition and a
     * week or two before the first competition because all the competition-specific appendages are being built
     * and assembled to it.
     * The <i>competition</i> robot is the index 0 entry in the list, so it does not require any changes to the
     * Roborio. The <i>practice</i> robot requires a jumper on the digital input port 5 that connects the signal
     * pin to ground. This convention was chosen to minimize the things that could go wrong on the competition robot.
     * These settings are loaded into {@link #ROBOT_SETTINGS_LIST} during {@link Robot#robotInit()}
     */
    public static final RobotSettings[] ROBOT_SETTINGS = {
            new A05Constants.RobotSettings(0, "Competition", 0.5461, 0.5461, 5.108, 4.085,
                    0.983, 5.041, 1.0, 0.9406),
            new RobotSettings(1, "Practice", 0.5969, 0.5969, 5.240, 5.654,
                    0.969, 5.039, 1.0, 0.9164)
    };

    public static final AutonomousPath[] AUTONOMOUS_PATHS = {
            //new AutonomousPath("Test", 10, "samplePath.json"),
            new AutonomousPath("Left", 0, "leftDoubleCoral.json"),
            new AutonomousPath("Middle", 0, "middleSingleCoral.json"),
            new AutonomousPath("Right", 10, "rightDoubleCoral.json")
    };

    public static final SendableChooser<AutonomousPath> AUTO_SELECTOR = new SendableChooser<>();

    public static void populateAutoSelector() {
        for(AutonomousPath path : AUTONOMOUS_PATHS) {
            if(AUTO_SELECTOR.getSelected() == null) {
                AUTO_SELECTOR.setDefaultOption(path.getName(), path);
            } else {
                AUTO_SELECTOR.addOption(path.getName(), path);
            }
        }

        AUTO_SELECTOR.onChange(Constants::setAuto);
    }

    public static AutonomousPath setAuto(AutonomousPath autonomousPath) {
        try {
            //autonomousPath = A05Constants.AUTONOMOUS_PATH_LIST.get(autoId);
            autonomousPath.load();
            SmartDashboard.putString("Autonomous", autonomousPath.getName());
        } catch (FileNotFoundException e) {
            SmartDashboard.putString("Autonomous",
                    String.format("Could not load path: '%s'", autonomousPath.getName()));
        }
        return autonomousPath;
    }

    public static final DriverSettings[] DRIVER_SETTINGS = {
            new DriverSettings("Salma", 0)
    };

    public static void setAprilTagPositionParametersDictionary() {
        aprilTagSetDictionary.put("left coral station", new AprilTagSet(new int[]{1}, new int[]{13}, new AngleD(AngleUnit.DEGREES, -126)));
        aprilTagSetDictionary.put("right coral station", new AprilTagSet(new int[]{2}, new int[]{12}, new AngleD(AngleUnit.DEGREES, 126)));
        aprilTagSetDictionary.put("close center reef", new AprilTagSet(new int[]{7}, new int[]{18}, new AngleD(AngleUnit.DEGREES, 0)));
        aprilTagSetDictionary.put("far center reef", new AprilTagSet(new int[]{10}, new int[]{21}, new AngleD(AngleUnit.DEGREES, 180)));
        aprilTagSetDictionary.put("close left reef", new AprilTagSet(new int[]{6}, new int[]{19}, new AngleD(AngleUnit.DEGREES, 60)));
        aprilTagSetDictionary.put("far left reef", new AprilTagSet(new int[]{11}, new int[]{20}, new AngleD(AngleUnit.DEGREES, 120)));
        aprilTagSetDictionary.put("close right reef", new AprilTagSet(new int[]{8}, new int[]{17}, new AngleD(AngleUnit.DEGREES, -60)));
        aprilTagSetDictionary.put("far right reef", new AprilTagSet(new int[]{9}, new int[]{22}, new AngleD(AngleUnit.DEGREES, -120)));
        aprilTagSetDictionary.put("all reef", new AprilTagSet(new int[]{6, 7, 8, 9, 10, 11}, new int[]{17, 18, 19, 20, 21, 22}));
        aprilTagSetDictionary.put("left cage", new AprilTagSet(new int[]{5}, new int[]{14}, new AngleD(AngleUnit.DEGREES, 0)));
        aprilTagSetDictionary.put("right cage", new AprilTagSet(new int[]{4}, new int[]{15}, new AngleD(AngleUnit.DEGREES, 0)));
        aprilTagSetDictionary.put("processor", new AprilTagSet(new int[]{3}, new int[]{16}, new AngleD(AngleUnit.DEGREES, 90)));
    }
}
