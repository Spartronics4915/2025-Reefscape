// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.commands.Autos;
import com.spartronics4915.frc2025.commands.DriveCommands.SwerveTeleopCommand;
import com.spartronics4915.frc2025.commands.DriveCommands.ChassisSpeedSuppliers;
import com.spartronics4915.frc2025.commands.DriveCommands.RotationIndependentControlCommand;
import com.spartronics4915.frc2025.subsystems.MotorSimulationSubsystem;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.NoteLocatorSim;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem mSwerveSubsystem = SwerveSubsystem.getInstance();

    private static final CommandXboxController mDriverController = new CommandXboxController(OI.kDriverControllerPort);

    private static final CommandXboxController mOperatorController = new CommandXboxController(OI.kOperatorControllerPort);
    
    private static final CommandXboxController mDebugController = new CommandXboxController(OI.kDebugControllerPort);

    public final TargetDetectorInterface mNoteDetector; //FIXME this never gets initalized

    public final MotorSimulationSubsystem mSim;

    private final SendableChooser<Command> autoChooser;
    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {

        if (RobotBase.isSimulation()) {
            mNoteDetector = new NoteLocatorSim(mSwerveSubsystem);
        } else {
            mNoteDetector = null;
        }

        mSim = new MotorSimulationSubsystem();
        ModeSwitchHandler.EnableModeSwitchHandler(); //TODO add any subsystems that extend ModeSwitchInterface

        // Configure the trigger bindings
        configureBindings();

        // Need to initialize this here after vision is configured.
        // Need to clean up initialization flow to make it more clear
        autoChooser = buildAutoChooser();
        
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        final SwerveTeleopCommand swerveTeleopCommand = new SwerveTeleopCommand(mDriverController);


        mDriverController.a().whileTrue(
            new RotationIndependentControlCommand(
                ChassisSpeedSuppliers.targetDetector(mNoteDetector::getClosestVisibleTarget, 360),
                () -> {
                    return ChassisSpeedSuppliers.computeVelocitiesFromController(mDriverController.getHID(), true, mSwerveSubsystem);
                }
            ));

        mSwerveSubsystem.setDefaultCommand(swerveTeleopCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // return Autos.driveToNote(swerveSubsystem, noteDetector);
        return autoChooser.getSelected();

    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("None", Commands.none());
        chooser.addOption("DriveToNote", Autos.driveToNote(mSwerveSubsystem, mNoteDetector));
        SmartDashboard.putData("Auto Chooser", chooser);

        return chooser;
    }

    public static CommandXboxController getDriveController(){return mDriverController;}
    public static CommandXboxController getOperatorController(){return mOperatorController;}
    public static CommandXboxController getDebugController(){return mDebugController;}

    
}
