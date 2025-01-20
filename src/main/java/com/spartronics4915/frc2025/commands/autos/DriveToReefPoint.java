package com.spartronics4915.frc2025.commands.autos;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.commands.ElementLocator;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class DriveToReefPoint {

    private final ElementLocator elementLocator;
    Field2d field;
    private final int targetTagID;
    private final Pose2d reefPoint, approachPoint;
    private final SwerveSubsystem swerve;

    public DriveToReefPoint(SwerveSubsystem swerve, ElementLocator elementLocator, int targetTagID) {

        this.elementLocator = elementLocator;
        this.targetTagID = targetTagID;
        this.swerve = swerve;

        if (RobotBase.isSimulation()) {
            field = new Field2d();
            SmartDashboard.putData("Field 2", field);
        }

        reefPoint = elementLocator.getLeftReefPoint(targetTagID);
        approachPoint = elementLocator.getApproachPoint(reefPoint, 1);
    }

    public Command generate() {
        Command command = Commands.none();

        if (RobotBase.isSimulation()) {
            command = command.andThen(() -> {
                field.getObject("TagPose").setPose(elementLocator.getLeftReefPoint(targetTagID));
                field.getObject("ApproachPoint").setPose(approachPoint);
                field.setRobotPose(swerve.getPose());

                System.out.println(elementLocator.getLeftReefPoint(targetTagID));
                System.out.println(swerve.getPose());
                System.out.println(approachPoint);
                

            });
        }
        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(5, 1);
        command = command.andThen(
                new DriveToPointCommand(approachPoint.getTranslation(), translationConstraints, 0.4, 0.5, swerve));

        return command;

    }

}
