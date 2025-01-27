// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025.commands;

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

    public static Command driveToNote(SwerveSubsystem swerve, TargetDetectorInterface detector) {

        return new AimDriveToTargetWIthTimeout(detector, swerve, 4, 0.5, 180).andThen(swerve.stopChassisCommand());
    }

    public static Command reverseForSeconds(SwerveSubsystem swerve, double seconds){
        return Commands.run(() -> swerve.driveFieldOriented(new ChassisSpeeds(ChassisSpeedSuppliers.shouldFlip() ? 1 : -1,0,0)), swerve).withTimeout(seconds);
    }

}
