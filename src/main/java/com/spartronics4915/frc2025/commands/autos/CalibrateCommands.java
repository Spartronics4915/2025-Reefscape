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

    private boolean gotInitialPose;

    Pose2d startVision;

    public CalibrateCommands(SwerveSubsystem swerve, VisionDeviceSubystem vision) {
        this.swerve = swerve;
        this.vision = vision;
        SmartDashboard.putNumber("calibrate/visionDist", 0);
        SmartDashboard.putNumber("calibrate/ratio", 0);
        field = new Field2d();
        SmartDashboard.putData("calibrate", field);
        field.getObject("start").setPose(Pose2d.kZero);
        field.getObject("vision").setPose(Pose2d.kZero);
        field.getObject("odom").setPose(Pose2d.kZero);
    }

    private boolean gotStartPose() {

        var poseVal = vision.getBotPose2dFromReefCamera();
        if (poseVal.isEmpty()) {
            return false;
        }
        startVision = poseVal.get();
        return true;
    }

    private void start() {

        field.getObject("start").setPose(startVision);
        swerve.setPose(startVision);

    }

    private void monitor() {
        Pose2d currOdometryPose = swerve.getPose();
        var visionVal = vision.getBotPose2dFromReefCamera();

        field.getObject("odom").setPose(currOdometryPose);
        double odomDist = currOdometryPose.getTranslation().getDistance(startVision.getTranslation());
        SmartDashboard.putNumber("odomDist", odomDist);

        if(visionVal.isPresent())
        {
            Pose2d currVisionPose = visionVal.get();
            field.getObject("vision").setPose(currVisionPose);

            double visionDist = currVisionPose.getTranslation().getDistance(startVision.getTranslation());
    
            SmartDashboard.putNumber("calibrate/visionDist", visionDist);
            SmartDashboard.putNumber("calibrate/ratio", odomDist/visionDist);
            }

    }
    public Command getCommand() {

        Command command = Commands.waitUntil(()->{return gotStartPose();});
        
        command = command.andThen(Commands.runOnce(()->{start();}));
        Command runCommand = Autos.reverseForSeconds(swerve, 10);
        Command monitorCommand = Commands.run(()->{monitor();});
        Command finalCommand = Commands.race(runCommand, monitorCommand);

        return command.andThen(finalCommand);
    }
}
