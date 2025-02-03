package com.spartronics4915.frc2025.commands;

import java.lang.management.PlatformLoggingMXBean;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class ElementLocator {
    

    private final AprilTagFieldLayout fieldLayout;

    private final double kDistanceFromTagToCoralMeters = .2;

    private final List<Integer> blueCoralTags = List.of(17,18,19, 20, 21, 22);
    private final List<Integer> redCoralTags = List.of(6, 7, 8, 9, 10,11);

    public ElementLocator() {

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public Pose2d getClosestReefPose(List<Integer> tagList, Translation2d robotPoint) {
        
        double minDist = -1;

        Pose2d bestPose = null;
        for(var i: tagList) {
            List<Pose2d> reefPoses = getReefPoses(i);
            for (Pose2d p: reefPoses) {
                double distToRobot = robotPoint.getDistance(p.getTranslation());
                if(distToRobot < minDist) {
                    bestPose = p;
                    distToRobot = minDist;
                }
            }

        }

        return bestPose;

    }

    public Pose2d getClosestReefPose(DriverStation.Alliance alliance, Translation2d robotPoint) {

        if(alliance == Alliance.Blue) {
            return getClosestReefPose(blueCoralTags, robotPoint);
        }
        else{
            return getClosestReefPose(redCoralTags, robotPoint);
        }
    }

    public List<Pose2d> getReefPoses(int tagID) {
        
        var poseResult = fieldLayout.getTagPose(tagID);

        Pose2d tagPose = poseResult.get().toPose2d();

        Translation2d tagLocation = tagPose.getTranslation();
        Translation2d offset = new Translation2d(0,-kDistanceFromTagToCoralMeters).rotateBy(tagPose.getRotation());
        Pose2d leftPose = new Pose2d(tagLocation.plus(offset), tagPose.getRotation());
        offset = new Translation2d(0,-kDistanceFromTagToCoralMeters).rotateBy(tagPose.getRotation());
        Pose2d rightPose = new Pose2d(tagLocation.plus(offset), tagPose.getRotation());

        return List.of(leftPose, rightPose);

    }

    public Pose2d getLeftReefPose(int tagID) {
        
        var poseResult = fieldLayout.getTagPose(tagID);

        Pose2d tagPose = poseResult.get().toPose2d();

        Translation2d tagLocation = tagPose.getTranslation();
        Translation2d offset = new Translation2d(0,-kDistanceFromTagToCoralMeters).rotateBy(tagPose.getRotation());
        Pose2d newPose = new Pose2d(tagLocation.plus(offset), tagPose.getRotation());
        return newPose;

    }


    public Pose2d getApproachPoint(Pose2d reefPoint, double distToPoint) {

        Translation2d pointOffset = new Translation2d(distToPoint, 0);
        var rotatedOffset = pointOffset.rotateBy(reefPoint.getRotation());
        var approachPoint = reefPoint.getTranslation().plus(rotatedOffset);

        return new Pose2d(approachPoint, reefPoint.getRotation().rotateBy(Rotation2d.k180deg));


    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
}
