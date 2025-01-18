package com.spartronics4915.frc2025.commands.autos;

import java.lang.Thread.State;

import com.spartronics4915.frc2025.commands.Autos;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class CalibrateCommands {
    
    public final SwerveSubsystem swerve;
    public final VisionDeviceSubystem vision;
    public Field2d field;

    Pose2d startVision;

    public CalibrateCommands(SwerveSubsystem swerve, VisionDeviceSubystem vision) {
        this.swerve = swerve;
        this.vision = vision;
    }


    private void start() {

        field = new Field2d();
        SmartDashboard.putData("calibrate", field);
        startVision = vision.getBotPose2dFromReefCamera().get();
        field.getObject("start").setPose(startVision);

    }

    private void monitor() {
        Pose2d currOdometryPose = swerve.getPose();
        Pose2d currVisionPose = vision.getBotPose2dFromReefCamera().get();

        field.getObject("odom").setPose(currOdometryPose);
        field.getObject("vision").setPose(currVisionPose);

        double visionDist = currVisionPose.getTranslation().getDistance(startVision.getTranslation());
        double odomDist = currOdometryPose.getTranslation().getDistance(startVision.getTranslation());

        SmartDashboard.putNumber("odomDist", odomDist);
        SmartDashboard.putNumber("visionDist", visionDist);
        SmartDashboard.putNumber("Ratio", odomDist/visionDist);

    }
    public Command getCommand() {

        Command command = Commands.runOnce(()->{start();});
        Command runCommand = Autos.reverseForSeconds(swerve, 5);
        Command monitorCommand = Commands.run(()->{monitor();});
        Command finalCommand = Commands.race(runCommand, monitorCommand);

        return command.andThen(finalCommand);
    }
}
