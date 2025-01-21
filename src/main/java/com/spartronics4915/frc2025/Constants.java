// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.LimelightModel;
import com.spartronics4915.frc2025.subsystems.vision.LimelightDevice.LimelightRole;
import com.spartronics4915.frc2025.util.Structures.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
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
    public static final class OI {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kDebugControllerPort = 2;


        public static final double kStickDeadband = 0.05;

        public static final double kDriverTriggerDeadband = 0.3;
        public static final double kOperatorTriggerDeadband = 0.3;
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

        public static final double kMaxSpeed = 5;
        public static final double kMaxAngularSpeed = kMaxSpeed * Math.PI / kChassisRadius;

        public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(5.0,0,0);
            public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);

            public enum PathplannerConfigs{
                PROGRAMMER_CHASSIS(new RobotConfig( // FIXME replace constants with more accurate values
                    Mass.ofBaseUnits(10, Kilogram), 
                    MomentOfInertia.ofBaseUnits(4, KilogramSquareMeters), // FIXME rough guess based on research
                    new ModuleConfig(
                        Distance.ofBaseUnits(4/2, Inches),
                        LinearVelocity.ofBaseUnits(5, MetersPerSecond),
                        1.19,
                        DCMotor.getNEO(1),
                        6.75,
                        Current.ofBaseUnits(40, Amps),
                        1
                    ),
                    new Translation2d(Distance.ofBaseUnits(12.25, Inches), Distance.ofBaseUnits(12.3125, Inches)),
                    new Translation2d(Distance.ofBaseUnits(12.25, Inches), Distance.ofBaseUnits(-12.3125, Inches)),
                    new Translation2d(Distance.ofBaseUnits(-12.25, Inches), Distance.ofBaseUnits(12.3125, Inches)),
                    new Translation2d(Distance.ofBaseUnits(-12.25, Inches), Distance.ofBaseUnits(-12.3125, Inches))
                ));

                public RobotConfig config;
    
                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }
        }

    }

    public static final class DriveCommandConstants {
        public static final PIDFConstants kAnglePIDConstants = new PIDFConstants(16, 0, 0, 0);
    }

    public static final class VisionConstants {
        public static final double kMaxAngularSpeed = 720;
        
        // Commenting this out for now because loading this is expensive and we want to have control over load times in auto.
        // public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        public static final LimelightConstants kLimelights[] = {
                new LimelightConstants("alex", LimelightModel.LIMELIGHT_3G, 11, LimelightRole.REEF),
                new LimelightConstants("randy", LimelightModel.LIMELIGHT_3, 12, LimelightRole.OBSERVER),
                new LimelightConstants("ben", LimelightModel.LIMELIGHT_3G, 13, LimelightRole.STATION)
        };
    }
}
