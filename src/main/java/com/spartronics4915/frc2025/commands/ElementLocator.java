package com.spartronics4915.frc2025.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ElementLocator {
    
    private final AprilTagFieldLayout fieldLayout;

    private final double kDistanceFromTagToCoralMeters = .2;

    public ElementLocator() {

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
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
