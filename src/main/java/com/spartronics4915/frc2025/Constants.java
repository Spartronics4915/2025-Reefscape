// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.spartronics4915.frc2025.subsystems.bling2.BlingLEDPattern;
import com.spartronics4915.frc2025.subsystems.bling2.BlingSegment;
import com.spartronics4915.frc2025.subsystems.bling2.BlingShow;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.PIDFConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class IntakeConstants {
        public static final int kMotorID = 12;

        public static final int kLaserCANID = 21;
        public static final int laserCANDistance = 110;

        public static final int smartCurrentLimit = 18;
        public static final int secondaryCurrentLimit = 20;

        public static final double kOpenLoopRampRate = 0.1;

        public static final double kLaserCanDebounce = 0.05;

        public static final EncoderConfig kEncoderConfig = new EncoderConfig()
            .positionConversionFactor(1/4.0)
            .velocityConversionFactor(1/4.0);

        public static final ClosedLoopConfig kCLConfig = new ClosedLoopConfig()
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0006, 0, 0.0);

        public static final SparkBaseConfig kMotorConfig = new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .apply(kCLConfig)
            .apply(kEncoderConfig)
            .openLoopRampRate(kOpenLoopRampRate)
            .smartCurrentLimit(smartCurrentLimit)
            .secondaryCurrentLimit(secondaryCurrentLimit);

        public enum IntakeSpeed {
            IN (-2500, -0.8),
            NEUTRAL (0, 0),
            OUT (2500, 0.8),
            FUNNEL_UNSTUCK(1800, 0.5);

            public final double intakeSpeed;
            public final double intakePercentage;
            
            private IntakeSpeed(double intakeSpeed, double intakePercentage) {
                this.intakeSpeed = intakeSpeed;
                this.intakePercentage = intakePercentage;
            }
        }
    }

    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kDebugControllerPort = 2;


        public static final double kStickDeadband = 0.05;
        public static final double kAngleStickDeadband = 0.25;
        public static final boolean kStartFieldRel = true;


        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;

    }

    public static final class ClimberConstants{

        public static final int motorID = 13;
        
        public static final double liftedAngle = 0.75;
        public static final double stowAngle = 0.25;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        //ff  
            //place holder values
            public static final Rotation2d kMinAngle = Rotation2d.fromRotations(0);
            public static final Rotation2d kMaxAngle = Rotation2d.fromRotations(1);

            public static final double kDt = 0.02;

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;



        public static final Constraints kConstraints = new Constraints(1.0, 1.0);

        public enum ClimberState {
        
            LIFTED(Rotation2d.fromDegrees(Constants.ClimberConstants.liftedAngle)),
            STOW(Rotation2d.fromDegrees(Constants.ClimberConstants.stowAngle)),;
        
            public final Rotation2d angle;

            private ClimberState(Rotation2d angle) {
                this.angle = angle;
            
            }

        }



    }

    public static final class Drive {
        public enum SwerveDirectories{
            NEO("swerve/neo"),
            PROGRAMMER_CHASSIS("swerve/programmer-chassis"),
            COMP_CHASSIS("swerve/comp-chassis");

            public String directory;

            private SwerveDirectories(String directory) {
                this.directory = directory;
            }
        }

        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5); //previously 5 (pathplanner max vel/acc divided by 2 as well)
        public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) * Math.PI / kChassisRadius);

        public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(5.0,0,0);
            public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);

            public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
                Drive.AutoConstants.kTranslationPID, 
                Drive.AutoConstants.kRotationPID
            );

            public enum PathplannerConfigs{
                PROGRAMMER_CHASSIS(new RobotConfig( // FIXME replace constants with more accurate values
                    Kilogram.of(10), 
                    KilogramSquareMeters.of(1.9387211145),
                    new ModuleConfig(
                        Inches.of(3.75/2.0),
                        MetersPerSecond.of(4),
                        1.00, //CHECKUP guess
                        DCMotor.getNEO(1),
                        6.75,
                        Amps.of(40),
                        1
                    ),
                    new Translation2d(Inches.of(12.625), Inches.of(12.5625)),
                    new Translation2d(Inches.of(12.125), Inches.of(-12.5)),
                    new Translation2d(Inches.of(-12), Inches.of(12.5)),
                    new Translation2d(Inches.of(-12.125), Inches.of(-12.4375))
                )),
                COMP_CHASSIS(new RobotConfig( // FIXME replace constants with more accurate values
                    Kilogram.of(125), 
                    KilogramSquareMeters.of(4.86247863),
                    new ModuleConfig(
                        Inches.of(3.75/2.0),
                        MetersPerSecond.of(4),
                        1.3, //CHECKUP guess
                        DCMotor.getNEO(1),
                        6.75,
                        Amps.of(40),
                        1
                    ),
                    new Translation2d(Inches.of(13.5), Inches.of(11.5)),
                    new Translation2d(Inches.of(13.625), Inches.of(-11.625)),
                    new Translation2d(Inches.of(-13.625), Inches.of(11.5)),
                    new Translation2d(Inches.of(-13.5), Inches.of(-10.4375))
            ));
;

                public RobotConfig config;
    
                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }

            public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
                Drive.AutoConstants.kTranslationPID, 
                Drive.AutoConstants.kRotationPID
            );

            public static final Time kAutoAlignPredict = Seconds.of(0.0);

            public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
            public static final Distance kPositionTolerance = Centimeter.of(1.0);
            public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(1);

            public static final Time kEndTriggerDebounce = Seconds.of(0.1);

            public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
            public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);


            public static final LinearVelocity kStationApproachSpeed = InchesPerSecond.of(5);
            public static final Time kStationApproachTimeout = Seconds.of(5);

            public static final PathConstraints kStartingPathConstraints = new PathConstraints(3, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.


            public static final PathConstraints kPathConstraints = new PathConstraints(2, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
        
            // X = side to side, Y = away from tag
            // public static final Translation2d kTagOffset = new Translation2d(0.10, 0.55); //TODO fix based off field cad

            public static final class StationVisualizationConstants {
                    public static final Pose2d kBlueLeft = new Pose2d(0.947, 7.447, Rotation2d.fromDegrees(-50));
                    public static final Pose2d kBlueRight = new Pose2d(0.947, 0.614, Rotation2d.fromDegrees(50));
                    public static final Pose2d kRedLeft = new Pose2d(16.603, 0.614, Rotation2d.fromDegrees(130));
                    public static final Pose2d kRedRight = new Pose2d(16.603, 7.447, Rotation2d.fromDegrees(-120));
            }
        }

    }

    public static final class DriveCommandConstants {
        public static final PIDFConstants kAnglePIDConstants = new PIDFConstants(5.0, 0.0, 0.0, 0);
    }

    public static final class OrientTowardsNearestPOIConstants {
        public static final Rotation2d REEF_OFFSET = Rotation2d.k180deg;
        public static final Translation2d REEF_CENTER_RED = new Translation2d(13.067, 4.031);
        public static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.471, 4.031);
        public static final double CORAL_STATION_ANGLE = 55;
        public static final Rotation2d BARGE_ROTATION = Rotation2d.kCCW_90deg;
        public static final Translation2d[] BARGE_RED_CAGE_POSITIONS = {
            new Translation2d(8.8, 3),
            new Translation2d(8.8, 1.9),
            new Translation2d(8.8, .8)
        };
        public static final Translation2d[] BARGE_BLUE_CAGE_POSITIONS = {
            new Translation2d(8.8, 7.2),
            new Translation2d(8.8, 6.1),
            new Translation2d(8.8, 5)
        };
    }

    public static final class BlingConstants {
        public static boolean LIGHTS_ENABLED = true;
        public static final boolean RUMBLE_ENABLED = true;

        public static final int BLING_BRIGHTNESS = RobotBase.isSimulation() ? 100 : 70; // 0-100
        public static final int FRAME_WAIT = 5; // Frames to wait before updating bling again

        public static final int BLING_LENGTH = 48;

        // Driver Communication Constants
        public static final double ARM_THRESHOLD = 10; // Degrees
        public static final double ELEVATOR_THRESHOLD = .1; // Meters
        public static final double BARGE_ALIGNMMENT_THRESHOLD = .2; // Distance that the robot can be, in meters, from the barge starting position.

        // Segment constants
        public static final BlingLEDPattern RAINBOW = BlingSegment.scrollingRainbow(BLING_LENGTH, 20);

        public static final BlingLEDPattern OFF = new BlingLEDPattern(LEDPattern.kOff, BLING_LENGTH);
        public static final BlingLEDPattern GOOD = BlingSegment.pulseColor(BLING_LENGTH, Color.kLime, .8);
        public static final BlingLEDPattern WARN = BlingSegment.pulseColor(BLING_LENGTH, Color.kYellow, .4);
        public static final BlingLEDPattern BAD = BlingSegment.pulseColor(BLING_LENGTH, Color.kRed, .2);

        public static final BlingLEDPattern RED = BlingSegment.solid(Color.kRed, BLING_LENGTH);
        public static final BlingLEDPattern ORANGE = BlingSegment.solid(Color.kOrange, BLING_LENGTH);
        public static final BlingLEDPattern YELLOW = BlingSegment.solid(Color.kYellow, BLING_LENGTH);
        public static final BlingLEDPattern GREEN = BlingSegment.solid(Color.kLime, BLING_LENGTH);
        public static final BlingLEDPattern BLUE = BlingSegment.solid(Color.kBlue, BLING_LENGTH);
        public static final BlingLEDPattern CYAN = BlingSegment.solid(Color.kCyan, BLING_LENGTH);
        public static final BlingLEDPattern PURPLE = BlingSegment.solid(Color.kMediumOrchid, BLING_LENGTH);
        public static final BlingLEDPattern WHITE = BlingSegment.solid(Color.kWhite, BLING_LENGTH);

        public static final BlingShow SHOW_SPARTRONICS = new BlingShow("bling/spartronics48.bling");
        public static final BlingShow SHOW_RAINBOW_FUN = new BlingShow("bling/rainbow48.bling");

        public static final BlingSegment MATCH_END = SHOW_RAINBOW_FUN;
    }

    public static final class VisionConstants {
        public static final double kMaxAngularSpeed = 720;
        public static final double kMaxSpeedForMegaTag1 = 0.5; //meters
        public static final double kMaxDistanceForMegaTag1 = 3.75; //meters
        public static final boolean kVisionDiagnostics = true;
        
        public static final Time newMegaTag1ReadingThreshold = Seconds.of(10);
        
        // Commenting this out for now because loading this is expensive and we want to have control over load times in auto.
        // public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        public static final LimelightConstants kLimelights[] = {
                new LimelightConstants("alex", LimelightModel.LIMELIGHT_3G, 11, LimelightRole.REEF),
                new LimelightConstants("randy", LimelightModel.LIMELIGHT_3, 12, LimelightRole.NOTHING),
                new LimelightConstants("ben", LimelightModel.LIMELIGHT_3G, 13, LimelightRole.REEF), //TODO: CHANGE
                new LimelightConstants("chucky", LimelightModel.LIMELIGHT_3, 14, LimelightRole.NOTHING),
                new LimelightConstants("doug", LimelightModel.LIMELIGHT_3, 15, LimelightRole.NOTHING)
        };

        public static final class StdDevConstants {
            public static final class MegaTag1 {
                public static final double kInitialValue = 0.3;
                public static final double kTagCountReward = 0.15;
                public static final double kAverageDistancePunishment = 0.1;
                public static final double kRobotSpeedPunishment = 0.15;
                public static final double kSingleTagPunishment = 0.3;
            }
            public static final class MegaTag2 {
                public static final double kInitialValue = 0.2;
                public static final double kAverageDistancePunishment = 0.075;
                public static final double kRobotSpeedPunishment = 0.25;
                public static final double kMultipleTagsBonus = 0.05;
            }
        }

        public enum LimelightModel {
            LIMELIGHT_3, LIMELIGHT_3G
        }
    
        public enum LimelightRole {
            NOTHING, REEF, ALIGN, STATION
        }

        public enum PoseEstimationMethod {
            MEGATAG_1, MEGATAG_2
        }
    }

    public static final class OdometryConstants {
        public static final double kMaxSwerveVisionPoseDifference = 1.0; //meters
    }

    public static final class ArmConstants {
        //I dont know the numbers yet so 0 is a place holder
        public enum ArmSubsystemState {

            EH(Rotation2d.fromDegrees(270)),
            INTAKE(Rotation2d.fromDegrees(225)),
            SCORE(Rotation2d.fromDegrees(180)),
            STOW(Rotation2d.fromDegrees(90));

            public Rotation2d angle;

            private ArmSubsystemState(Rotation2d angle) {
                this.angle = angle;
            }

        }
        
        public static final int kArmMotorID = 11;
        public static final double kPositionConversionFactor = 1 * 0.03888888888888;
        public static final double kVelocityConversionFactor = 1 * 0.03888888888888;
        
        public static final double kDt = 0.02;

        public static final Constraints kConstraints = new Constraints(4.5, 3.5 / 2.0); //8.0, 10
        public static final int kPeriodMs = 0;

        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        
        //The values set here are placeholders for sim
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(5);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(330);

        public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(270);

        public static final SlotConfigs kPIDConfigs = new SlotConfigs()
            .withKP(200)
            .withKI(0.0)
            .withKD(0.0);

        public static final CurrentLimitsConfigs kCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(20);
        public static final FeedbackConfigs kFeedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(25.7143)
        ;

    }

    public static final class ElevatorConstants {

        public enum ElevatorSubsystemState {

            STOW(0),
            L1(0.1),
            L3(0.3),
            L4(1.1);

            public double meter;

            private ElevatorSubsystemState(double meter) {
                this.meter = meter;
            }
        }

        public static final int elevatorMotorID = 9; //left motor
        public static final int elevatorFollowerID = 10; //right motor
        public static final boolean motorInverted = true;
        public static final boolean followerInverted = true;
        public static final double motorPositionConversionFactor = (1/20.0) * 0.14044 * 2;
        public static final double motorVelocityConversionFactor = (1/20.0) * 0.14044 * 2;
        public static final int motorSmartCurrentLimit = 35; //18
        public static final int motorSecondaryCurrentLimit = 40; //20
        public static final int followerSmartCurrentLimit = 35;
        public static final int followerSecondaryCurrentLimit = 40;

        public static final double dt = 0.02;

        public static final Constraints constraints = new Constraints(3.0, 3.5); //12, 7.5

        public static final double minHeight = 0;
        public static final double maxHeight = 1.24;

        // Not using elevator feedforward constants for now, so just commenting them out.
        
        // public static final double kS = 0.0;
        // public static final double kG = 0.0;
        // public static final double kV = 0.0;
        // public static final double kA = 0.0;

        public static final class motorPIDConstants {
            public static final double kP = 16;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class DynamicsConstants {
        public static final Angle kArmAngleTolerance = Degrees.of(1);
        public static final double kElevatorHeightTolerance = Inches.of(1).in(Meters);

        public static final double kSafeElevHeightForSwerve = 0.4;


        public static final Angle kSafeArmAngle = Degrees.of(90); //TODO this is currently straight up, this might change
        public static final Angle kMoveableArmAngle = Degrees.of(83.801389); //used in cos math, so this is equivalent to ~80 degrees either side of the left horizon //TODO this is currently straight up, this might change

        public static final Angle kRemoveAlgaeArmAngle = Degrees.of(11.6);
    
        public static final double kMinSafeElevHeight = 0.385; //previously 4.361// height of the elevator for when the arm is stowed and needs to move

        public static final double kScoreLaserCanDebounce = 0.1; //seconds

        public static final int kFunnelLaserCanID = 20;
        public static final Distance funnelLCTriggerDist = Meters.of(0.2);

    }

    public static final class WinchClimberConstants {
        //angles have 0 being horizantally away from the chassis, with clockwise rotation (when looking at the robot from the front) being positive

        public enum WinchSpeeds{
            ENGAGE(0.0), //speed which it'll rotate to move and engage the cage
            RETRACT(0.0), //speed which it'll rotate to bring the cage down
            ;

            public final double speed;

            private WinchSpeeds(double speed) {
                this.speed = speed;
            }
        }
        
        public static final int kMotorID = 13;

        private static final EncoderConfig kEncoderConfig = new EncoderConfig()
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0)
        ;

        public static final SparkBaseConfig kMotorConfig = new SparkMaxConfig()
            .smartCurrentLimit(35)
            .secondaryCurrentLimit(40)
            .inverted(false)
            .openLoopRampRate(0.25)
            .idleMode(IdleMode.kBrake)
            .apply(kEncoderConfig)
        ;


        public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(90.0); //angle at the start of the match
        public static final Rotation2d kEngagedAngle = Rotation2d.fromDegrees(270.0); //angle to engage the cage
        public static final Rotation2d kRetractedAngle = Rotation2d.fromDegrees(180.0); //desired angle at the end of the match


    }
    @SuppressWarnings("unused")
    private static final Optional<Integer> kGender = Optional.empty();
}
