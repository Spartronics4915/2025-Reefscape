// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.PIDFConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

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
            .smartCurrentLimit(smartCurrentLimit)
            .secondaryCurrentLimit(secondaryCurrentLimit);

        public enum IntakeSpeed {
            IN (500),
            NEUTRAL (0),
            OUT (-1000);

            public final double intakeSpeed;
            
            private IntakeSpeed(double intakeSpeed) {
                this.intakeSpeed = intakeSpeed;
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
            PROGRAMMER_CHASSIS("swerve/programmer-chassis");

            public String directory;

            private SwerveDirectories(String directory) {
                this.directory = directory;
            }
        }

        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5);
        public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) * Math.PI / kChassisRadius);

        public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(5.0,0,0);
            public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);

            public enum PathplannerConfigs{
                PROGRAMMER_CHASSIS(new RobotConfig( // FIXME replace constants with more accurate values
                    Kilogram.of(10), 
                    KilogramSquareMeters.of(1.9387211145),
                    new ModuleConfig(
                        Inches.of(3.75/2.0),
                        MetersPerSecond.of(5),
                        1.00, //CHECKUP guess
                        DCMotor.getNEO(1),
                        6.75,
                        Amps.of(40),
                        1
                    ),
                    new Translation2d(Inches.of(12.25), Inches.of(12.3125)),
                    new Translation2d(Inches.of(12.25), Inches.of(-12.3125)),
                    new Translation2d(Inches.of(-12.25), Inches.of(12.3125)),
                    new Translation2d(Inches.of(-12.25), Inches.of(-12.3125))
                ));

                public RobotConfig config;
    
                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }

            public static final PathConstraints kPathConstraints = new PathConstraints(1.75, 1.75, 1/2 * Math.PI, 1 * Math.PI); // The constraints for this path.
        
            // X = side to side, Y = away from tag
            public static final Translation2d kTagOffset = new Translation2d(0.10, 0.55); //TODO fix based off field cad
        }

    }

    public static final class DriveCommandConstants {
        public static final PIDFConstants kAnglePIDConstants = new PIDFConstants(5.0, 0.0, 0.0, 0);
    }

    public static final class VisionConstants {
        public static final double kMaxAngularSpeed = 720;
        public static final double kMaxSpeedForMegaTag1 = 0.5; //meters
        public static final boolean kVisionDiagnostics = true;
        
        // Commenting this out for now because loading this is expensive and we want to have control over load times in auto.
        // public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        public static final LimelightConstants kLimelights[] = {
                new LimelightConstants("alex", LimelightModel.LIMELIGHT_3G, 11, LimelightRole.REEF),
                new LimelightConstants("randy", LimelightModel.LIMELIGHT_3, 12, LimelightRole.ALIGN),
                new LimelightConstants("ben", LimelightModel.LIMELIGHT_3G, 13, LimelightRole.STATION),
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
                public static final double kInitialValue = 0.1;
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

            INTAKE(Rotation2d.fromDegrees(0)),
            SCORE(Rotation2d.fromDegrees(30)),
            EH(Rotation2d.fromDegrees(90)),
            STOW(Rotation2d.fromDegrees(180));

            public Rotation2d angle;

            private ArmSubsystemState(Rotation2d angle) {
                this.angle = angle;
            }

        }
        
        public static final int kArmMotorID = 11;
        public static final int kPositionConversionFactor = 1;
        public static final int kVelocityConversionFactor = 1;
        
        public static final double kDt = 0.02;

        public static final Constraints kConstraints = new Constraints(1.0, 1.0);
        public static final int kPeriodMs = 0;

        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.02;
        public static final double kA = 0.0;
        
        //The values set here are placeholders for sim
        public static final Rotation2d kMinAngle = Rotation2d.fromRotations(-3);
        public static final Rotation2d kMaxAngle = Rotation2d.fromRotations(3);

        public static final SlotConfigs kPIDConfigs = new SlotConfigs()
            .withKP(1)
            .withKI(0.0)
            .withKD(0.0);
    }

    public static final class ElevatorConstants {

        public enum ElevatorSubsystemState {

            STOW(0),
            L1(0.1),
            L3(0.2),
            L4(0.3);

            public double meter;

            private ElevatorSubsystemState(double meter) {
                this.meter = meter;
            }
        }

        public static final int elevatorMotorID = 9; //left motor
        public static final int elevatorFollowerID = 10; //right motor
        public static final boolean motorInverted = false;
        public static final boolean followerInverted = true;
        public static final double motorPositionConversionFactor = (1/20.0) * 0.14044 * 2;
        public static final double motorVelocityConversionFactor = (1/20.0) * 0.14044 * 2;
        public static final int motorSmartCurrentLimit = 13;
        public static final int motorSecondaryCurrentLimit = 15;
        public static final int followerSmartCurrentLimit = 13;
        public static final int followerSecondaryCurrentLimit = 15;

        public static final double dt = 0.02;

        public static final Constraints constraints = new Constraints(1.0, 1.0);

        public static final double minHeight = 0;
        public static final double maxHeight = 0.5;

        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final class motorPIDConstants {
            public static final double kP = 0.25;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
}

