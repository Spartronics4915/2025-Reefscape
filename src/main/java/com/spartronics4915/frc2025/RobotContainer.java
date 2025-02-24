// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.spartronics4915.frc2025.Constants.ArmConstants.ArmSubsystemState;
import com.spartronics4915.frc2025.Constants.ElevatorConstants.ElevatorSubsystemState;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.Constants.BlingConstants;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.commands.Autos;
import com.spartronics4915.frc2025.commands.ComplexAutoChooser;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory;
import com.spartronics4915.frc2025.commands.ElementLocator;
import com.spartronics4915.frc2025.commands.VariableAutos;
import com.spartronics4915.frc2025.commands.Autos.AutoPaths;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory.DynaPreset;
import com.spartronics4915.frc2025.commands.autos.AlignToReef;
import com.spartronics4915.frc2025.commands.autos.DriveToReefPoint;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchHeight;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchSide;
import com.spartronics4915.frc2025.commands.VariableAutos.FieldBranch;
import com.spartronics4915.frc2025.commands.VariableAutos.ReefSide;
import com.spartronics4915.frc2025.commands.VariableAutos.StationSide;
import com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers;
import com.spartronics4915.frc2025.commands.drive.RotationIndependentControlCommand;
import com.spartronics4915.frc2025.commands.drive.SwerveTeleopCommand;
import com.spartronics4915.frc2025.subsystems.ClimberSubsystem;
import com.spartronics4915.frc2025.subsystems.MechanismRenderer;
import com.spartronics4915.frc2025.subsystems.MotorSimulationSubsystem;
import com.spartronics4915.frc2025.subsystems.OdometrySubsystem;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.bling2.*;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.SimVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;

import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.shouldFlip;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
@Logged
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Drive.SwerveDirectories.COMP_CHASSIS);

    private static final CommandXboxController driverController = new CommandXboxController(OI.kDriverControllerPort);

    private static final CommandXboxController operatorController = new CommandXboxController(
        OI.kOperatorControllerPort);
        
    private static final CommandXboxController debugController = new CommandXboxController(OI.kDebugControllerPort);

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    private final ElementLocator elementLocator = new ElementLocator();
    private VisionDeviceSubystem visionSubsystem = null;
    private OdometrySubsystem odometrySubsystem = null;
    
    public final IntakeSubsystem intakeSubsystem;
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final ClimberSubsystem climberSubsystem;

    
    public final DynamicsCommandFactory dynamics;

    public SwerveTeleopCommand swerveTeleopCommand = null;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    
    public final BlingSubsystem blingSubsystem;
    
    private AlignToReef alignmentCommandFactory = null;
    private VariableAutos variableAutoFactory = null;

    @NotLogged
    private final SendableChooser<Command> autoChooser;

    @NotLogged
    private final ComplexAutoChooser complexAutoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        intakeSubsystem = new IntakeSubsystem();
        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        climberSubsystem = new ClimberSubsystem();

        dynamics = new DynamicsCommandFactory(armSubsystem, elevatorSubsystem, intakeSubsystem);

        ModeSwitchHandler.EnableModeSwitchHandler(
            intakeSubsystem,
            armSubsystem,
            elevatorSubsystem
        ); //TODO add any subsystems that extend ModeSwitchInterface

        MechanismRenderer.generateRenderer(
            elevatorSubsystem::getDesiredPosition, 
            () -> armSubsystem.getTargetPosition().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "Target Position"
        );

        MechanismRenderer.generateRenderer(
            () -> Meters.of(elevatorSubsystem.getPosition()), 
            () -> armSubsystem.getPosition().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "Current Position"
        );

        MechanismRenderer.generateRenderer(
            () -> elevatorSubsystem.getSetpoint(), 
            () -> armSubsystem.getSetpoint().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "setpoints"
        );

        if (swerveSubsystem != null) {
            swerveTeleopCommand = new SwerveTeleopCommand(driverController, swerveSubsystem);
            alignmentCommandFactory = new AlignToReef(swerveSubsystem, fieldLayout);
            variableAutoFactory = new VariableAutos(alignmentCommandFactory, dynamics);
            if (RobotBase.isSimulation()) {
                visionSubsystem = new SimVisionSubsystem(swerveSubsystem);
            } else {
                visionSubsystem = new LimelightVisionSubsystem(swerveSubsystem, elementLocator.getFieldLayout());
                ModeSwitchHandler.EnableModeSwitchHandler((LimelightVisionSubsystem) visionSubsystem);
            }
    
            odometrySubsystem = new OdometrySubsystem(visionSubsystem, swerveSubsystem);
        }

        // Configure the trigger bindings
        configureBindings();

        complexAutoChooser = new ComplexAutoChooser(variableAutoFactory, 4);

        // Need to initialize this here after vision is configured.
        // Need to clean up initialization flow to make it more clear
        autoChooser =
                buildAutoChooser();

        blingSubsystem = new BlingSubsystem(0, new DriverCommunication(BlingConstants.BLING_LENGTH, swerveSubsystem));
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
        

        //#region driver controls

        if (swerveSubsystem != null) {
            swerveSubsystem.setDefaultCommand(swerveTeleopCommand);

            driverController.leftStick().onTrue(Commands.runOnce(() -> {
                swerveTeleopCommand.resetHeadingOffset();
            }));

            driverController.leftTrigger()
                .whileTrue(
                    Commands.run(swerveSubsystem::lockModules, swerveSubsystem)
                    .withName("X Brake Swerve")
                );

            //this is a approximate version, we can do something more advanced by placing points at the center of the reef sides, then detecting which side it's closest to based on it's position
            driverController.rightTrigger().whileTrue(
                new RotationIndependentControlCommand(
                    ChassisSpeedSuppliers.gotoAngle(ChassisSpeedSuppliers.orientTowardsNearestPOI(swerveSubsystem), swerveSubsystem),
                    ChassisSpeedSuppliers.getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem),
                    swerveSubsystem
                )
                .withName("Orient Towards Nearest POI")
            );

            driverController.b().toggleOnTrue(
                Commands.startEnd(
                    () -> {swerveTeleopCommand.setFieldRelative(!OI.kStartFieldRel);},
                    () -> {swerveTeleopCommand.setFieldRelative(OI.kStartFieldRel);}
                )
                .withName("Toggle Field Relative")
            );

            driverController.a().onTrue(
                Commands.defer(() -> {
                    return Commands.runOnce(() -> {
                        swerveTeleopCommand.setHeadingOffset(swerveSubsystem.getPose().getRotation());
                    });
                }, Set.of())
            );

            driverController.leftBumper().whileTrue(
                alignmentCommandFactory.generateCommand(BranchSide.LEFT).finallyDo((boolean interrupted) -> {
                    dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
                })
                .withName("Align Left Branch")
            );
    
            driverController.rightBumper().whileTrue(
                alignmentCommandFactory.generateCommand(BranchSide.RIGHT).finallyDo((boolean interrupted) -> {
                    dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
                })
                .withName("Align Right Branch")
            );
        }

        //#endregion

        //#region automated controls

        dynamics.hasScoredTrigger.onTrue(dynamics.stow());

        new Trigger(intakeSubsystem::detect).and(DriverStation::isTeleop).debounce(0.02).onTrue(
            Commands.parallel(
                dynamics.stow()
            ));

        new Trigger(dynamics::funnelDetect).onTrue(
            dynamics.intake()
        );

        //#endregion

        //#region Operator Controls

        operatorController.rightTrigger().onTrue( //whileTrue
            dynamics.score()
        );/*.onFalse(Commands.parallel(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL),
            dynamics.stow()
        ));*/

        operatorController.leftTrigger().onTrue(dynamics.stow());

        operatorController.back().onTrue(dynamics.loadStow()); //windows button

        operatorController.y().onTrue(dynamics.operatorScore(DynaPreset.L4));

        operatorController.x().onTrue(dynamics.operatorScore(DynaPreset.L3));

        operatorController.b().onTrue(dynamics.operatorScore(DynaPreset.L2));

        operatorController.start().onTrue(dynamics.intake()); //menu button

        operatorController.povUp().whileTrue(elevatorSubsystem.manualMode(0.002));

        operatorController.povDown().whileTrue(elevatorSubsystem.manualMode(-0.002));

        operatorController.povLeft().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(-0.3)));

        operatorController.povRight().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(0.3)));

        //#endregion

        SmartDashboard.putData("stowLoad", dynamics.loadStow());
        SmartDashboard.putData("stowPreScore", dynamics.prescoreStow());
        SmartDashboard.putData("stow", dynamics.stow());
        SmartDashboard.putData("score", dynamics.score());

        SmartDashboard.putData("L1", dynamics.gotoScore(DynaPreset.L1));
        SmartDashboard.putData("L2", dynamics.gotoScore(DynaPreset.L2));
        SmartDashboard.putData("L3", dynamics.gotoScore(DynaPreset.L3));
        SmartDashboard.putData("L4", dynamics.gotoScore(DynaPreset.L4));

        SmartDashboard.putData("intake", dynamics.intake());

        SmartDashboard.putNumber("elevator setpoint", 0);
        SmartDashboard.putNumber("arm setpoint", 270);

        SmartDashboard.putData("ManualElevator", Commands.defer(() -> {
            return elevatorSubsystem.setSetPointCommand(SmartDashboard.getNumber("elevator setpoint", 0.0));
        }, Set.of()));
        SmartDashboard.putData("ManualArm", Commands.defer(() -> {
            return armSubsystem.setSetpointCommand(Rotation2d.fromDegrees(SmartDashboard.getNumber("arm setpoint", 270.0)));
        }, Set.of()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // return Autos.driveToNote(swerveSubsystem, noteDetector);
        // return new DriveToReefPoint(swerveSubsystem, elementLocator, 11).generate();
        // return complexAutoChooser.getAuto();
        return autoChooser.getSelected();

    }

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();

        NamedCommands.registerCommand("print", Commands.print("ping"));


        chooser.addOption("GartronicsDynamicsScoreL3", Commands.sequence(
            dynamics.stow(),
            dynamics.blockingIntake(),
            dynamics.gotoScore(BranchHeight.L3.preset),
            dynamics.autoScore(BranchHeight.L3.preset),
            dynamics.stow()
        ));

        chooser.addOption("GartronicsDynamicsScoreL2", Commands.sequence(
            dynamics.stow(),
            dynamics.blockingIntake(),
            dynamics.gotoScore(BranchHeight.L2.preset),
            dynamics.autoScore(BranchHeight.L2.preset),
            dynamics.stow()
        ));

        chooser.addOption("GartronicsDynamicsScoreL4", Commands.sequence(
            dynamics.stow(),
            dynamics.blockingIntake(),
            dynamics.gotoScore(BranchHeight.L4.preset),
            dynamics.autoScore(BranchHeight.L4.preset),
            dynamics.stow()
        ));
        

        chooser.setDefaultOption("None", Commands.none());

        if (swerveSubsystem != null) {
            chooser.addOption("ReverseLeave", Autos.reverseForSeconds(swerveSubsystem, 3));
            chooser.addOption("Drive to Reef Point", new DriveToReefPoint(swerveSubsystem, elementLocator, 11).generate());
            chooser.addOption("M-R debug straight", new PathPlannerAuto("M-R straight debug"));
            chooser.addOption("M-R debug curve", new PathPlannerAuto("M-R curve debug"));
            chooser.addOption("M-R Circle", new PathPlannerAuto("Circle move debug"));
            chooser.addOption("Reef loop debug", new PathPlannerAuto("Reef loop debug"));
            chooser.addOption("Leave", new PathPlannerAuto("Leave Auto"));

            chooser.addOption("Align with move", Commands.sequence(
                variableAutoFactory.generateAutoCycle(FieldBranch.A, StationSide.LEFT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.C, StationSide.LEFT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.E, StationSide.LEFT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.G, StationSide.LEFT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.I, StationSide.LEFT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.K, StationSide.LEFT, BranchHeight.L4)
            ));

            chooser.addOption("Align Mirror with move", Commands.sequence(
                variableAutoFactory.generateAutoCycle(FieldBranch.A, StationSide.RIGHT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.C, StationSide.RIGHT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.E, StationSide.RIGHT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.G, StationSide.RIGHT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.I, StationSide.RIGHT, BranchHeight.L4),
                variableAutoFactory.generateAutoCycle(FieldBranch.K, StationSide.RIGHT, BranchHeight.L4)
            ));

            var variableAuto = Commands.defer(complexAutoChooser::getAuto, Set.of(swerveSubsystem));
            variableAuto.setName("variableAuto");

            chooser.addOption("Create auto...", variableAuto);
        }

        chooser.onChange((c) -> {
            SmartDashboard.putBoolean("Using Variable Auto?", c.getName() == "variableAuto");
        });


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

    public static AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

}
