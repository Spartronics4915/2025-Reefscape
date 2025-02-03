package com.spartronics4915.frc2025.commands.drive;

import java.util.function.BooleanSupplier;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.commands.ElementLocator;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ReefModeCommands {

    static public Pose2d currReefPoint = null;

    private static void chooseReefPointImpl(SwerveDrivePoseEstimator poseEstimator, ElementLocator locator) {

        Pose2d currPose = poseEstimator.getEstimatedPosition();
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d nearestReefPoint = locator.getClosestReefPose(DriverStation.getAlliance().get(),
                    currPose.getTranslation());

            currReefPoint = nearestReefPoint;
        }
    }

    public static Command chooseReefPoint(SwerveDrivePoseEstimator poseEstimator, ElementLocator locator) {

        return Commands.runOnce(() -> chooseReefPointImpl(poseEstimator, locator));
    }

    public static BooleanSupplier aimedAtReefPoint(Translation2d reefPoint, SwerveDrivePoseEstimator poseEstimator,
            double closeEnoughDegrees) {

        return () -> Math
                .abs(ChassisSpeedSuppliers.getFieldAngleBetween(poseEstimator.getEstimatedPosition().getTranslation(),
                        currReefPoint.getTranslation()).getDegrees()) < closeEnoughDegrees;
    }

    public static Command aimAtReefPoint(SwerveSubsystem swerveSubsystem) {

        if (currReefPoint == null)

        {
            return Commands.none();
        } else {
            return new RotationIndependentControlCommand(
                    ChassisSpeedSuppliers.gotoAngle(
                            () -> ChassisSpeedSuppliers.getFieldAngleBetween(swerveSubsystem.getPose().getTranslation(),
                                    currReefPoint.getTranslation()),
                            swerveSubsystem),
                    () -> new ChassisSpeeds(),
                    swerveSubsystem)
                            .until(aimedAtReefPoint(currReefPoint.getTranslation(),
                                    swerveSubsystem.getInternalSwerve().swerveDrivePoseEstimator, 5))
                            .andThen(swerveSubsystem.stopChassisCommand()).andThen(Commands.none());
        }
    }
}
