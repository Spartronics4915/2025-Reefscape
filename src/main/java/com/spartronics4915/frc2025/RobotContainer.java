// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.commands.Autos;
import com.spartronics4915.frc2025.commands.ElementLocator;
import com.spartronics4915.frc2025.commands.autos.DriveToReefPoint;
import com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers;
import com.spartronics4915.frc2025.commands.drive.RotationIndependentControlCommand;
import com.spartronics4915.frc2025.commands.drive.SwerveTeleopCommand;
import com.spartronics4915.frc2025.subsystems.MotorSimulationSubsystem;
import com.spartronics4915.frc2025.subsystems.OdometrySubsystem;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.Bling.BlingSegment;
import com.spartronics4915.frc2025.subsystems.Bling.BlingShow;
import com.spartronics4915.frc2025.subsystems.Bling.BlingSubsystem;
import com.spartronics4915.frc2025.subsystems.Bling.DriverCommunication;
import com.spartronics4915.frc2025.subsystems.vision.SimVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;

import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.shouldFlip;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Drive.SwerveDirectories.PROGRAMMER_CHASSIS);

    private static final CommandXboxController driverController = new CommandXboxController(OI.kDriverControllerPort);

    private static final CommandXboxController operatorController = new CommandXboxController(
            OI.kOperatorControllerPort);

    private static final CommandXboxController debugController = new CommandXboxController(OI.kDebugControllerPort);

    private final ElementLocator elementLocator = new ElementLocator();
    private final VisionDeviceSubystem visionSubsystem;
    private final OdometrySubsystem odometrySubsystem;

    // ******** Simulation entries
    public final MotorSimulationSubsystem mechanismSim;
    // ********

    public final SwerveTeleopCommand swerveTeleopCommand = new SwerveTeleopCommand(driverController, swerveSubsystem);
    // Replace with CommandPS4Controller or CommandJoystick if needed

    public final BlingSubsystem blingSubsystem;

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        mechanismSim = new MotorSimulationSubsystem();
        ModeSwitchHandler.EnableModeSwitchHandler(swerveSubsystem); //TODO add any subsystems that extend ModeSwitchInterface

        if (RobotBase.isSimulation()) {
            visionSubsystem = new SimVisionSubsystem(swerveSubsystem);
        } else {
            visionSubsystem = new LimelightVisionSubsystem(swerveSubsystem, elementLocator.getFieldLayout());
            ModeSwitchHandler.EnableModeSwitchHandler((LimelightVisionSubsystem) visionSubsystem);
        }
        blingSubsystem = new BlingSubsystem(0, new DriverCommunication(42, swerveSubsystem));

        odometrySubsystem = new OdometrySubsystem(visionSubsystem, swerveSubsystem);

        // Configure the trigger bindings
        configureBindings();

        // Need to initialize this here after vision is configured.
        // Need to clean up initialization flow to make it more clear
        autoChooser =

                buildAutoChooser();

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

        swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(driverController, swerveSubsystem));


        //switch field and robot relative
        driverController.a().onTrue(Commands.defer(() -> {return Commands.runOnce(
                () -> swerveTeleopCommand.setFieldRelative(!swerveTeleopCommand.getFieldRelative())
            );}
        ,Set.of()));

        driverController.b().onTrue(
            Commands.defer(() -> {
                return Commands.runOnce(() -> {
                    swerveTeleopCommand.setHeadingOffset(swerveSubsystem.getPose().getRotation());
                });
            }, Set.of())
        );

        driverController.leftStick().onTrue(Commands.runOnce(() -> {
            swerveTeleopCommand.resetHeadingOffset();
        }));
        
        driverController.leftTrigger()
            .whileTrue(
                Commands.run(swerveSubsystem::lockModules, swerveSubsystem)
            );


        //this is a approximate version, we can do something more advanced by placing points at the center of the reef sides, then detecting which side it's closest to based on it's position
        driverController.rightTrigger().whileTrue(
            new RotationIndependentControlCommand(
                ChassisSpeedSuppliers.gotoAngle(() -> ChassisSpeedSuppliers.getFieldAngleBetween(swerveSubsystem.getPose().getTranslation(), 
                    shouldFlip() ? new Translation2d(13.073, 4): new Translation2d(4.5, 4)
                ), swerveSubsystem),
                ChassisSpeedSuppliers.getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem),
                swerveSubsystem
            )
        );

        swerveSubsystem.setDefaultCommand(swerveTeleopCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // return Autos.driveToNote(swerveSubsystem, noteDetector);
        // return new DriveToReefPoint(swerveSubsystem, elementLocator, 11).generate();
        return autoChooser.getSelected();

    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();

        NamedCommands.registerCommand("print", Commands.print("ping"));

        chooser.setDefaultOption("None", Commands.none());
        chooser.addOption("ReverseLeave", Autos.reverseForSeconds(swerveSubsystem, 3));
        chooser.addOption("Drive to Reef Point", new DriveToReefPoint(swerveSubsystem, elementLocator, 11).generate());
        chooser.addOption("M-R debug straight", new PathPlannerAuto("M-R straight debug"));
        chooser.addOption("M-R debug curve", new PathPlannerAuto("M-R curve debug"));
        chooser.addOption("M-R Circle", new PathPlannerAuto("Circle move debug"));
        chooser.addOption("Reef loop debug", new PathPlannerAuto("Reef loop debug"));

        SmartDashboard.putData("Auto Chooser", chooser);

        return chooser;
    }

    public static CommandXboxController getDriveController() {
        return driverController;
    }

    public static CommandXboxController getOperatorController() {
        return operatorController;
    }

    public static CommandXboxController getDebugController() {
        return debugController;
    }

}
