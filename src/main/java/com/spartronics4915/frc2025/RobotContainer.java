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
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.ClimberSpeeds;
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.WinchSpeeds;
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
import com.spartronics4915.frc2025.commands.autos.AlignToReef.FieldBranchSide;
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
import com.spartronics4915.frc2025.subsystems.WinchClimber;
import com.spartronics4915.frc2025.subsystems.bling2.*;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.SimVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.CoralSim;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;
import com.spartronics4915.frc2025.util.RumbleFeedbackHandler.RumbleController;
import com.spartronics4915.frc2025.util.RumbleFeedbackHandler.RumbleFeedback;
import com.spartronics4915.frc2025.util.RumbleFeedbackHandler.RumblePresets;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;

import static com.spartronics4915.frc2025.Constants.DynamicsConstants.kElevatorHeightTolerance;
import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.climberCamMode;
import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.shouldFlip;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Drive.SwerveDirectories.COMP_CHASSIS);

    private static final CommandXboxController driverController = new CommandXboxController(OI.kDriverControllerPort);

    private static final CommandXboxController operatorController = new CommandXboxController(
        OI.kOperatorControllerPort);
        
    private static final CommandXboxController debugController = DriverStation.isFMSAttached() ? null : new CommandXboxController(OI.kDebugControllerPort);

    private enum Rumble{
        DRIVER(driverController),
        OPERATOR(operatorController);
        // DEBUG(debugController);

        public final RumbleController controller;

        private Rumble(CommandXboxController controller) {
            this.controller = new RumbleController(controller);
        }
    }

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    private final ElementLocator elementLocator = new ElementLocator();
    private VisionDeviceSubystem visionSubsystem = null;
    private OdometrySubsystem odometrySubsystem = null;
    
    public final IntakeSubsystem intakeSubsystem;
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final WinchClimber climberSubsystem;

    public boolean isTeleopAutoScoringEnabled = true; 

    private final BooleanPublisher autoScoreEnabledPub = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("AutoScoringEnabled").publish();
    
    public final DynamicsCommandFactory dynamics;

    public SwerveTeleopCommand swerveTeleopCommand = null;
    // Replace with CommandPS4Controller or CommandJoystick if needed
    
    public final BlingSubsystem blingSubsystem;
    
    private AlignToReef alignmentCommandFactory = null;
    private VariableAutos variableAutoFactory = null;

    private final SendableChooser<Command> autoChooser;


    private final ComplexAutoChooser complexAutoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        intakeSubsystem = new IntakeSubsystem();
        armSubsystem = new ArmSubsystem(intakeSubsystem);
        elevatorSubsystem = new ElevatorSubsystem(intakeSubsystem);
        climberSubsystem = new WinchClimber();

        dynamics = new DynamicsCommandFactory(armSubsystem, elevatorSubsystem, intakeSubsystem);

        if (RobotBase.isSimulation()) CoralSim.setup(swerveSubsystem, intakeSubsystem);

        ModeSwitchHandler.EnableModeSwitchHandler(
            intakeSubsystem,
            armSubsystem,
            elevatorSubsystem,
            climberSubsystem,
            swerveSubsystem
        ); 

        MechanismRenderer.generateRenderer(
            elevatorSubsystem::getDesiredPosition, 
            () -> armSubsystem.getTargetPosition().getMeasure(), 
            () -> intakeSubsystem.setpoint, 
            intakeSubsystem::detect,
            intakeSubsystem::hasAlgae,
            "Target Position"
        );

        // MechanismRenderer.generateRenderer(
        //     () -> Meters.of(elevatorSubsystem.getPosition()), 
        //     () -> armSubsystem.getPosition().getMeasure(), 
        //     intakeSubsystem::getSpeed, 
        //     intakeSubsystem::detect,
        //     "Current Position"
        // );

        // MechanismRenderer.generateRenderer(
        //     () -> elevatorSubsystem.getSetpoint(), 
        //     () -> armSubsystem.getSetpoint().getMeasure(), 
        //     () -> RPM.of(intakeSubsystem.setpoint), 
        //     intakeSubsystem::detect,
        //     "setpoints"
        // );

        if (swerveSubsystem != null) {
            swerveTeleopCommand = new SwerveTeleopCommand(driverController, swerveSubsystem);
            alignmentCommandFactory = new AlignToReef(swerveSubsystem);
            variableAutoFactory = new VariableAutos(alignmentCommandFactory, dynamics, swerveSubsystem);
            if (RobotBase.isSimulation()) {
                visionSubsystem = new SimVisionSubsystem(swerveSubsystem);
            } else {
                visionSubsystem = new LimelightVisionSubsystem(swerveSubsystem, getFieldLayout());
                ModeSwitchHandler.EnableModeSwitchHandler((LimelightVisionSubsystem) visionSubsystem);
            }
    
            odometrySubsystem = new OdometrySubsystem(visionSubsystem, swerveSubsystem);
        }

        // Configure the trigger bindings
        configureBindings();

        complexAutoChooser = new ComplexAutoChooser(variableAutoFactory, 3, swerveSubsystem);

        // Need to initialize this here after vision is configured.
        // Need to clean up initialization flow to make it more clear
        autoChooser =
                buildAutoChooser();

        //blingSubsystem = new BlingSubsystem(0, Constants.BlingConstants.ORANGE); 
        DriverCommunication driverCommunication = new DriverCommunication(BlingConstants.BLING_LENGTH, swerveSubsystem, armSubsystem, elevatorSubsystem, dynamics, visionSubsystem);
        blingSubsystem = new BlingSubsystem(0, driverCommunication);

        AlignToReef.warmup();
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

            driverController.rightStick().toggleOnTrue(
                Commands.startEnd(
                    () -> {ChassisSpeedSuppliers.climberCamMode = true;},
                    () -> {ChassisSpeedSuppliers.climberCamMode = false;}
                )
                .withName("Toggle Climber Cam Mode")
            );

            driverController.a().onTrue(
                Commands.defer(() -> {
                    return Commands.runOnce(() -> {
                        swerveTeleopCommand.setHeadingOffset(swerveSubsystem.getPose().getRotation());
                    });
                }, Set.of())
            );

            driverController.leftBumper().whileTrue(
                alignmentCommandFactory.generateCommand(FieldBranchSide.LEFT)//.finallyDo((boolean interrupted) -> {
                //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
                // })
                .withName("Align Left Branch")
            );
    
            driverController.rightBumper().whileTrue(
                alignmentCommandFactory.generateCommand(FieldBranchSide.RIGHT)//.finallyDo((boolean interrupted) -> {
                //     dynamics.gotoLastInputtedScore().onlyIf(() -> !interrupted);
                // })
                .withName("Align Right Branch")
            );

            driverController.start().whileTrue(
                alignmentCommandFactory.generateCommand(FieldBranchSide.MIDDLE)
                .withName("Align Middle Branch")
            );

            ChassisSpeeds driverNudgeUp = new ChassisSpeeds(0.25, 0, 0);
            ChassisSpeeds driverNudgeLeft = new ChassisSpeeds(0, 0.25, 0);
            ChassisSpeeds driverNudgeRight = new ChassisSpeeds(0, -0.25, 0);
            ChassisSpeeds driverNudgeDown = new ChassisSpeeds(-0.25, 0, 0);

            driverController.povUp().whileTrue(
                Commands.run(() -> {
                    swerveSubsystem.drive(climberCamMode ? driverNudgeLeft : driverNudgeUp);
                })
            );

            driverController.povLeft().whileTrue(
                Commands.run(() -> {
                    swerveSubsystem.drive(climberCamMode ? driverNudgeDown : driverNudgeLeft);
                })
            );

            driverController.povRight().whileTrue(
                Commands.run(() -> {
                    swerveSubsystem.drive(climberCamMode ? driverNudgeUp : driverNudgeRight);
                })
            );

            driverController.povDown().whileTrue(
                Commands.run(() -> {
                    swerveSubsystem.drive(climberCamMode ? driverNudgeRight : driverNudgeDown);
                })
            );
        }

        //#endregion

        //#region Rumble

        if (OI.RUMBLE_ENABLED) {
            // Score is currently not used, maybe later?
            // dynamics.hasScoredTrigger.onTrue(
            //     Rumble.DEBUG.controller.timedRumble(RumblePresets.PRESET0L.rumble, 1.0)
            // );

            new Trigger(dynamics::funnelDetect).onTrue(
                Rumble.DRIVER.controller.timedRumble(RumblePresets.DRIVER_FUNNEL.rumble, OI.rumbleTime)
            );

            new Trigger(dynamics::isCoralInArm).onTrue(
                Rumble.OPERATOR.controller.timedRumble(RumblePresets.OPERATOR_INTAKE.rumble, OI.rumbleTime)
            );
        }

        //#endregion

        //#region automated controls

        new Trigger(dynamics::canAutoScore).and(DriverStation::isTeleop).and(() -> isTeleopAutoScoringEnabled).onTrue(Commands.sequence(
            dynamics.score()
        ));

        dynamics.hasScoredTrigger.and(DriverStation::isTeleop).onTrue(dynamics.queueLoadStow());

        new Trigger(intakeSubsystem::detect).and(DriverStation::isTeleop)
            .debounce(0.02).onTrue(
                Commands.parallel(
                    dynamics.stow()
                ).withName("auto stowing (trigger))")
                .onlyIf(dynamics::hasNotJustScored)
            );

        new Trigger(dynamics::funnelDetect).onTrue(
            dynamics.intake()
        );

        //#endregion

        //#region Operator Controls

        operatorController.rightTrigger().onTrue( //whileTrue
            Commands.parallel(
                dynamics.score(),
                Commands.print("scoring")
            )
        );/*.onFalse(Commands.parallel(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL),
            dynamics.stow()
        ));*/

        operatorController.leftTrigger().onTrue(dynamics.stow());

        operatorController.back().onTrue(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.FUNNEL_UNSTUCK)
        ).onFalse(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN)
        ); //windows button

        operatorController.y().onTrue(Commands.defer(() -> {
            return(dynamics.operatorScore(intakeSubsystem.hasAlgae() ? DynaPreset.BARGE : DynaPreset.L4));
        }, Set.of()));

        operatorController.x().onTrue(dynamics.operatorScore(DynaPreset.L3));

        operatorController.b().onTrue(dynamics.operatorScore(DynaPreset.L2));

        operatorController.a().onTrue(Commands.defer(() -> {
            return(dynamics.operatorScore(intakeSubsystem.hasAlgae() ? DynaPreset.PROCESSOR : DynaPreset.L1));
        }, Set.of()));

        operatorController.start().onTrue(dynamics.intake()); //menu button

        autoScoreEnabledPub.accept(isTeleopAutoScoringEnabled);

        operatorController.rightStick().onTrue(climberSubsystem.unSpoolWinch());

        // operatorController.rightStick().whileTrue(
        //     Commands.repeatingSequence(
        //         Commands.sequence(
        //             intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT),
        //             Commands.waitSeconds(.25)
        //         ).onlyIf(() -> !intakeSubsystem.detect()),
        //         Commands.sequence(
        //             intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN),
        //             Commands.waitSeconds(.25)
        //         ).onlyIf(() -> !intakeSubsystem.detect())
        //     )
        //     ).onFalse(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN).onlyIf(() -> !intakeSubsystem.detect()));

        Trigger leftStickUp = new Trigger(() -> (operatorController.getLeftY() < (-1 + OI.kPaddleTolerance)) && ((Math.abs(operatorController.getLeftX()) < OI.kPaddleTolerance) || operatorController.getLeftX() < (-1 + OI.kPaddleTolerance))); //top left paddle
        Trigger leftStickLeft = new Trigger(() -> (operatorController.getLeftX() < (-1 + OI.kPaddleTolerance)) && ((Math.abs(operatorController.getLeftY()) < OI.kPaddleTolerance) || operatorController.getLeftY() < (-1 + OI.kPaddleTolerance))); //bottom left paddle
        Trigger rightStickUp = new Trigger(() -> (operatorController.getRightY() < (-1 + OI.kPaddleTolerance)) && ((Math.abs(operatorController.getRightX()) < OI.kPaddleTolerance) || operatorController.getRightX() < (-1 + OI.kPaddleTolerance))); //top right paddle
        Trigger rightStickLeft = new Trigger(() -> (operatorController.getRightX() < (-1 + OI.kPaddleTolerance)) && ((Math.abs(operatorController.getRightY()) < OI.kPaddleTolerance) || operatorController.getRightY() < (-1 + OI.kPaddleTolerance))); //bottom right paddle

        Trigger algaeSafety = leftStickUp;
        
        rightStickUp.and(algaeSafety).onTrue(
            Commands.defer(() -> {
                Pose2d closestAprilTag = AlignToReef.getClosestReefAprilTag(swerveSubsystem.getPose());
                int index = AlignToReef.allReefTagPoses.indexOf(closestAprilTag);
                final DynaPreset algaeScoreHeight;
                switch (index) {
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                    case 9:
                    case 11:
                        algaeScoreHeight = DynaPreset.ALGAE_HIGH;
                        break;
                    case 0:
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                    case 10:
                        algaeScoreHeight = DynaPreset.ALGAE_LOW;
                        break;
                    default:
                        algaeScoreHeight = DynaPreset.ALGAE_HIGH;
                        break;
                }
                System.out.println(algaeScoreHeight);
                System.out.println(index);
                return dynamics.gotoScore(algaeScoreHeight).alongWith(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.ALGAE_INTAKE));
            }, Set.of())
        );

        operatorController.leftStick().onTrue(climberSubsystem.invertOperatorClimberControls());

        operatorController.leftBumper().onTrue(climberSubsystem.operatorClimberWinchCommand(true))
                                       .onFalse(climberSubsystem.operatorClimberWinchCommand(false));

        operatorController.rightBumper().onTrue(climberSubsystem.operatorClimberArmCommand(true))
                                        .onFalse(climberSubsystem.operatorClimberArmCommand(false))
                                        .onTrue(dynamics.gotoClimb());
        

        SmartDashboard.putData("setPreset1", armSubsystem.setMechanismAngleCommand(Rotation2d.fromDegrees(270)));

        SmartDashboard.putData("preset1Arm", armSubsystem.presetCommand(ArmSubsystemState.EH));

        SmartDashboard.putData("preset1elevator", elevatorSubsystem.presetCommand(ElevatorSubsystemState.STOW));
        SmartDashboard.putData("preset2elevator", elevatorSubsystem.presetCommand(ElevatorSubsystemState.L1));
        SmartDashboard.putData("preset3elevator", elevatorSubsystem.presetCommand(ElevatorSubsystemState.L3));
        SmartDashboard.putData("preset4elevator", elevatorSubsystem.presetCommand(ElevatorSubsystemState.L4));

        SmartDashboard.putData("Stow", dynamics.stow());
        SmartDashboard.putData("Stow Load", dynamics.loadStow());
        SmartDashboard.putData("Stow Prescore", dynamics.prescoreStow());
        SmartDashboard.putData("Climb", dynamics.gotoClimb());
        SmartDashboard.putData("L4", dynamics.gotoScore(DynaPreset.L4));
        SmartDashboard.putData("L3", dynamics.gotoScore(DynaPreset.L3));
        SmartDashboard.putData("L2", dynamics.gotoScore(DynaPreset.L2));
        SmartDashboard.putData("L1", dynamics.gotoScore(DynaPreset.L1));
        SmartDashboard.putData("Barge", dynamics.gotoScore(DynaPreset.BARGE));
        SmartDashboard.putData("Processor", dynamics.gotoScore(DynaPreset.PROCESSOR));
        SmartDashboard.putData("Launch", dynamics.gotoScore(DynaPreset.LAUNCH));

        SmartDashboard.putData("Score", dynamics.score());
        SmartDashboard.putData("Climber: stop", climberSubsystem.stopArmCommand());
        SmartDashboard.putData("Climber: Engage", climberSubsystem.setClimberSpeedsCommand(ClimberSpeeds.ENGAGE));
        SmartDashboard.putData("Climber: Retract", climberSubsystem.setClimberSpeedsCommand(ClimberSpeeds.RETRACT)); 
        SmartDashboard.putData("Climb: Move Arm", dynamics.gotoClimb());

        SmartDashboard.putData("Toggle Auto Score", Commands.defer(() -> {
            return Commands.runOnce(() -> {
                isTeleopAutoScoringEnabled = !isTeleopAutoScoringEnabled;
                autoScoreEnabledPub.accept(isTeleopAutoScoringEnabled);
            });
        }, Set.of()));

        SmartDashboard.putData("Reset Dynamics", dynamics.resetDynamics());

    
        if (debugController != null) {
            debugController.b().onTrue(Commands.runOnce(() -> LimelightVisionSubsystem.setMegaTag1Override(true)))
                               .onFalse(Commands.runOnce(() -> LimelightVisionSubsystem.setMegaTag1Override(false)));

            debugController.x().onTrue(Commands.runOnce(() -> LimelightVisionSubsystem.setDiscardMeasurements(true)))
                               .onFalse(Commands.runOnce(() -> LimelightVisionSubsystem.setDiscardMeasurements(false)));
        }

        operatorController.povUp().whileTrue(elevatorSubsystem.manualMode(0.002));
    
        operatorController.povDown().whileTrue(elevatorSubsystem.manualMode(-0.002));
    
        operatorController.povLeft().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(-0.3)));
    
        operatorController.povRight().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(0.3)));
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

        chooser.setDefaultOption("None", Commands.none());

        if (swerveSubsystem != null) {
            var variableAuto = Commands.defer(complexAutoChooser::getAuto, Set.of(swerveSubsystem));
            variableAuto.setName("variableAuto");

            chooser.setDefaultOption("Create auto...", variableAuto);

            // chooser.addOption("ReverseLeave", Autos.reverseForSeconds(swerveSubsystem, 3));
            // chooser.addOption("Drive to Reef Point", new DriveToReefPoint(swerveSubsystem, elementLocator, 11).generate());
            // chooser.addOption("M-R debug straight", new PathPlannerAuto("M-R straight debug"));
            // chooser.addOption("M-R debug curve", new PathPlannerAuto("M-R curve debug"));
            // chooser.addOption("M-R Circle", new PathPlannerAuto("Circle move debug"));
            // chooser.addOption("Reef loop debug", new PathPlannerAuto("Reef loop debug"));
            chooser.addOption("Leave", new PathPlannerAuto("Leave Auto"));

            chooser.addOption("Drive Forwards", Autos.driveForward(swerveSubsystem));

            chooser.addOption("Test Single Run", Commands.sequence(
                    dynamics.loadStow(),
                    dynamics.blockingIntake(),
                    Commands.defer(complexAutoChooser::getSingleRun, Set.of(swerveSubsystem))
            ));

            chooser.addOption("one time", Commands.sequence(
                variableAutoFactory.generateAutoCycle(FieldBranch.D, StationSide.RIGHT, BranchHeight.L4)
            ));

            chooser.addOption("Align with move", Commands.sequence(
                variableAutoFactory.generateAutoCycle(FieldBranch.A, StationSide.LEFT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.C, StationSide.LEFT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.E, StationSide.LEFT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.G, StationSide.LEFT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.I, StationSide.LEFT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.K, StationSide.LEFT, BranchHeight.L2)
            ));

            chooser.addOption("Align Mirror with move", Commands.sequence(
                variableAutoFactory.generateAutoCycle(FieldBranch.A, StationSide.RIGHT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.C, StationSide.RIGHT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.E, StationSide.RIGHT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.G, StationSide.RIGHT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.I, StationSide.RIGHT, BranchHeight.L2),
                variableAutoFactory.generateAutoCycle(FieldBranch.K, StationSide.RIGHT, BranchHeight.L2)
            ));
        }

        chooser.onChange(RobotContainer::postIfUsingVariableAutos);

        SmartDashboard.putData("Auto Chooser", chooser);

        postIfUsingVariableAutos(chooser.getSelected());

        return chooser;
    }

    private static void postIfUsingVariableAutos(Command c) {
        SmartDashboard.putBoolean("Using Variable Auto?", c.getName() == "variableAuto");
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
