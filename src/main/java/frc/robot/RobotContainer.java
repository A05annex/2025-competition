// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05RobotContainer;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends A05RobotContainer
{
    // The robot's subsystems and commands are defined here...
    // NavX, DriveSubsystem, DriveXbox have already been made in A05RobotContainer
    @SuppressWarnings("FieldCanBeLocal")
	private final SpeedCachedSwerve speedCachedSwerve = SpeedCachedSwerve.getInstance();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        super();

        speedCachedSwerve.setDriveSubsystem(driveSubsystem);
        speedCachedSwerve.setCacheLength(1000);
        speedCachedSwerve.setLatencyOffset(0.0);

        speedCachedSwerve.setDriveGeometry(robotSettings.length, robotSettings.width,
                robotSettings.rf, robotSettings.rr,
                robotSettings.lf, robotSettings.lr,
                robotSettings.maxSpeedCalibration);

        // finish swerve drive initialization for this specific robt.
        driveCommand = new DriveCommand(speedCachedSwerve);

        driveSubsystem.setDefaultCommand(driveCommand);

        EndEffectorSubsystem.getInstance().setDefaultCommand(new CoralCenterCommand());

        // Which robot is this? competition or spare/prototype
        int robotId = A05Constants.readRobotID();
        robotSettings = A05Constants.ROBOT_SETTINGS_LIST.get(robotId);

        // Configure the button bindings
        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        driveBack.onTrue(new InstantCommand(navx::initializeHeadingAndNav)); // Reset the NavX field relativity

        driveB.whileTrue(new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.SAFE::goTo));
        altB.whileTrue(new InstantCommand(ElevatorSubsystem.ELEVATOR_POSITION.SAFE::goTo));

        // End Effector spin while held
        driveY.onTrue(new InstantCommand(EndEffectorSubsystem.getInstance()::spin)).onFalse(new InstantCommand(EndEffectorSubsystem.getInstance()::stopAll));
        altStart.onTrue(new InstantCommand(EndEffectorSubsystem.getInstance()::spin)).onFalse(new InstantCommand(EndEffectorSubsystem.getInstance()::stopAll));
     // altStart = elevatorOverride

        altRightBumper.onTrue(new InstantCommand(AlgaeSubsystem.getInstance()::spin)).onFalse(new InstantCommand(AlgaeSubsystem.getInstance()::stop));
        altBack.onTrue(new InstantCommand(AlgaeSubsystem.getInstance()::reverse)).onFalse(new InstantCommand(AlgaeSubsystem.getInstance()::stop));

        altY.whileTrue(new AlgaeDescoreCommandGroup());

        altX.toggleOnTrue(new HumanIntakeCommand());
        driveX.toggleOnTrue(new HumanIntakeCommand());

        altA.whileTrue(new AllCoralScoreCommandGroup());

        /*
        driveLeftBumper = left station heading lock
        driveRightBumper = right station heading lock

        driveA = auto reef heading Lock

        altLeftBumper = reef heading lock
         */
    }
}
