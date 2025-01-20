// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025.commands;

import com.spartronics4915.frc2025.commands.drive.AimDriveToTargetWIthTimeout;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {

    public static Command driveToNote(SwerveSubsystem swerve, TargetDetectorInterface detector) {

        return new AimDriveToTargetWIthTimeout(detector, swerve, 4, 0.5, 180).andThen(swerve.stopChassisCommand());
    }

}
