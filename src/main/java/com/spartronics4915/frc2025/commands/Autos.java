// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025.commands;

import java.io.IOException;
import java.security.InvalidParameterException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.spartronics4915.frc2025.commands.autos.DriveToPointCommand;
import com.spartronics4915.frc2025.commands.drive.AimDriveToTargetWIthTimeout;
import com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command moveToPointAuto(SwerveSubsystem swerve) {
        Translation2d targetPose = new Translation2d(2.9, 7);
        Constraints driveConstraints = new Constraints(2, 2);
        return new DriveToPointCommand(targetPose, driveConstraints, 0.2, 0.1, swerve);
    }

    public enum AutoPaths{
        CORAL_ONE("Coral-1"),
        ONE_CORAL("1-Coral"),
        CORAL_TWO("Coral-2"),
        TWO_CORAL("2-Coral"),
        CORAL_THREE("Coral-3"),
        THREE_CORAL("3-Coral"),
        CORAL_FOUR("Coral-4"),
        FOUR_CORAL("4-Coral"),
        CORAL_FIVE("Coral-5"),
        FIVE_CORAL("5-Coral"),
        CORAL_SIX("Coral-6"),
        SIX_CORAL("6-Coral"),
        ;
        public final String pathName;

        //TODO create Mirroring so that we can switch different coral stations intuitively
        //TODO create "getReverse", ie Coral-2 reversed is 2-Coral

        /**
         * @return The inverse of the current path (return to approach and vise versa)
         */
        public AutoPaths getReverse(){
            switch (this) {
                case CORAL_ONE: return ONE_CORAL;
                case CORAL_TWO: return TWO_CORAL;
                case CORAL_THREE: return THREE_CORAL;
                case CORAL_FOUR: return FOUR_CORAL;
                case CORAL_FIVE: return FIVE_CORAL;
                case CORAL_SIX: return SIX_CORAL;
                case ONE_CORAL: return CORAL_ONE;
                case TWO_CORAL: return CORAL_TWO;
                case THREE_CORAL: return CORAL_THREE;
                case FOUR_CORAL: return CORAL_FOUR;
                case FIVE_CORAL: return CORAL_FIVE;
                case SIX_CORAL: return CORAL_SIX;
            }
            return this;
        }

        /**
         * @return The apporach / return path for the mirrored side 
         */
        public AutoPaths getMirror(){
            switch (this) {
                case CORAL_ONE: return CORAL_ONE;
                case CORAL_TWO: return TWO_CORAL;
                case CORAL_THREE: return THREE_CORAL;
                case ONE_CORAL: return ONE_CORAL;
                case TWO_CORAL: return CORAL_TWO;
                case THREE_CORAL: return CORAL_THREE;
            }
            return this;
        }

        private AutoPaths(String path) {
            pathName = path;
        }
    }

    public static Command getAutoPathCommand(AutoPaths path){
        return getAutoPathCommand(path, false);
    }

    public static PathPlannerPath getAutoPath(AutoPaths pathChoice, boolean mirrored){
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathChoice.pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            throw new InvalidParameterException("invalid path");
        }

        if(mirrored) path = path.mirrorPath();
        return path;
    }

    public static Command getAutoPathCommand(AutoPaths pathChoice, boolean mirrored){
        return AutoBuilder.followPath(getAutoPath(pathChoice, mirrored));
    }

    public static Command driveToNote(SwerveSubsystem swerve, TargetDetectorInterface detector) {

        return new AimDriveToTargetWIthTimeout(detector, swerve, 4, 0.5, 180).andThen(swerve.stopChassisCommand());
    }

    public static Command reverseForSeconds(SwerveSubsystem swerve, double seconds){
        return Commands.run(() -> swerve.driveFieldOriented(new ChassisSpeeds(ChassisSpeedSuppliers.shouldFlip() ? 1 : -1,0,0)), swerve).withTimeout(seconds);
    }


}
