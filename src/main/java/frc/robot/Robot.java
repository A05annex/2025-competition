// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05Robot;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.RobotPosition;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.photonvision.PhotonCamera;

import java.util.Collections;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends A05Robot {
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        super.robotInit();

        Constants.setSparkConfig(true,false);

        // Set the drive constants that are specific to this swerve geometry.
        // Some drive geometry is passed in RobotContainer's constructor
        Constants.setDriveOrientationKp(Constants.DRIVE_ORIENTATION_kP);

        Constants.setPrintDebug(false);

        // update dictionary with all needed values
        Constants.setAprilTagPositionParametersDictionary();
        // Load the robot settings list
        Collections.addAll(A05Constants.ROBOT_SETTINGS_LIST,Constants.ROBOT_SETTINGS);
        // Load the autonomous path list
        Collections.addAll(A05Constants.AUTONOMOUS_PATH_LIST,Constants.AUTONOMOUS_PATHS);
        // Load the driver list
        Collections.addAll(A05Constants.DRIVER_SETTINGS_LIST,Constants.DRIVER_SETTINGS);

        // Set the camera correction functions. This accounts for error in the reported distances from PhotonVision
        Constants.CAMERA.setXCorrectionFunction(Constants::xCorrectionFunction);
        Constants.CAMERA.setYCorrectionFunction(Constants::yCorrectionFunction);

        InferredRobotPosition.setCamera(Constants.CAMERA);


        Constants.ELEVATOR_ANALOG_ENCODER.setInverted(true);


        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        setRobotContainer(new RobotContainer());


        SmartDashboard.putData("L1", new InstantCommand());
        SmartDashboard.putData("L2", new InstantCommand());
        SmartDashboard.putData("L3", new InstantCommand());
        SmartDashboard.putData("AGI", new InstantCommand());
        SmartDashboard.putData("HPI", new InstantCommand());
        SmartDashboard.putData("LOW ALGAE", new InstantCommand());
        SmartDashboard.putData("HIGH ALGAE", new InstantCommand());
        SmartDashboard.putData("SAFE", new InstantCommand());

        InstantCommand
                L1 = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.L1::goTo),
                L2 = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.L2::goTo),
                L3 = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.L3::goTo),
                AGI = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.AGI::goTo),
                HPI = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.HPI::goTo),
                LOW_ALGAE = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_LOW::goTo),
                HIGH_ALGAE = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.ALGAE_HIGH::goTo),
                SAFE = new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.SAFE::goTo);


        L1.setName("L1");
        L2.setName("L2");
        L3.setName("L3");
        AGI.setName("AGI");
        HPI.setName("HPI");
        LOW_ALGAE.setName("LOW ALGAE");
        HIGH_ALGAE.setName("HIGH ALGAE");
        SAFE.setName("SAFE");



        SmartDashboard.putData("L1", L1);
        SmartDashboard.putData("L2", L2);
        SmartDashboard.putData("L3", L3);
        SmartDashboard.putData("AGI", AGI);
        SmartDashboard.putData("HPI", HPI);
        SmartDashboard.putData("LOW ALGAE", LOW_ALGAE);
        SmartDashboard.putData("HIGH ALGAE", HIGH_ALGAE);
        SmartDashboard.putData("SAFE", SAFE);


    }
    
    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    public void enableInit() {

    }
    
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {
        // Sets up autonomous command in A05Robot
        super.autonomousInit();

        enableInit();
    }

    @Override
    public void teleopInit()
    {
        // Cancels autonomous command
        super.teleopInit();

        enableInit();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        SmartDashboard.putNumber("elevator", ElevatorSubsystem.getInstance().getPosition());

        SmartDashboard.putNumber("encoder", Constants.ELEVATOR_ANALOG_ENCODER.get());
        SmartDashboard.putNumber("correctedEncoder", ElevatorSubsystem.getInstance().getCorrectedEncoder());

        SmartDashboard.putNumber("algae", AlgaeSubsystem.getInstance().getVelocity());

        SmartDashboard.putBoolean("front", Constants.frontSensor());
        SmartDashboard.putBoolean("back", Constants.backSensor());

        SmartDashboard.putString("D_PAD", Constants.getDPad(A05Constants.ALT_XBOX).toString());


        if(Constants.CAMERA.hasTargets(Constants.aprilTagSetDictionary.get("close center reef"))) {
            SmartDashboard.putNumber("X", Constants.CAMERA.getXFromLastTarget(Constants.aprilTagSetDictionary.get("close center reef")));
            SmartDashboard.putNumber("Y", Constants.CAMERA.getYFromLastTarget(Constants.aprilTagSetDictionary.get("close center reef")));
            SmartDashboard.putNumber("IRPX", InferredRobotPosition.getRobotPosition("close center reef").x);
            SmartDashboard.putNumber("IRPY", InferredRobotPosition.getRobotPosition("close center reef").y);
            SmartDashboard.putNumber("RPX", RobotPosition.getRobotPosition("close center reef").x);
            SmartDashboard.putNumber("RPY", RobotPosition.getRobotPosition("close center reef").y);
        }
    }

    @Override
    public void disabledPeriodic() {
        //SmartDashboard.putNumber("Heading", NavX.getInstance().getHeadingInfo().expectedHeading.getDegrees());
        //DriveSubsystem.getInstance().printAllAngles();

    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();

        // Example SmartDashboard telemetry call
        SmartDashboard.putNumber("elevator", ElevatorSubsystem.getInstance().getPosition());
    }


    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
