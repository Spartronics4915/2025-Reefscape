package com.spartronics4915.frc2025.commands.autos;

import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kPathConstraints;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kTagOffset;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.AprilTagRegion;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AlignToReef {

    public enum BranchSide{
        LEFT,
        RIGHT;
    } 
    
    private final SwerveSubsystem mSwerve;

    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();

    public AlignToReef(SwerveSubsystem mSwerve, AprilTagFieldLayout field) {
        this.mSwerve = mSwerve;

        Arrays.stream(AprilTagRegion.REEF.blue()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.REEF.red()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });
    }

    public Command generateCommand(BranchSide side) {
        var waypoint = getBranchPathWaypoint(side);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(mSwerve.getPose().getTranslation(), getVelocityHeading(mSwerve.getFieldVelocity(), waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return Commands.none();
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            kPathConstraints,
            null, 
            new GoalEndState(0.0, getBranchRotation(mSwerve))
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
    

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (Math.abs(cs.vxMetersPerSecond) < 0.01 && Math.abs(cs.vyMetersPerSecond) < 0.01 ) {
            var diff =  mSwerve.getPose().minus(target).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    /**
     * 
     * @return Pathplanner waypoint with
     */
    private Pose2d getBranchPathWaypoint(BranchSide side){
        var nearest = getClosestBranch(side, mSwerve);
        return new Pose2d(
            nearest.getTranslation(),
            nearest.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    /**
     * 
     * @return target rotation for the robot when it reaches the final waypoint
     */
    private Rotation2d getBranchRotation(SwerveSubsystem swerve){
        return getClosestReefAprilTag(swerve.getPose()).getRotation().rotateBy(Rotation2d.k180deg);
    }

    public static Pose2d getClosestBranch(BranchSide side, SwerveSubsystem swerve){
        Pose2d tag = getClosestReefAprilTag(swerve.getPose());
        
        var translation = tag.getTranslation().plus(
            new Translation2d(
                kTagOffset.getY(),
                kTagOffset.getX() * (side == BranchSide.LEFT ? -1 : 1)
            ).rotateBy(tag.getRotation())
        );

        return new Pose2d(
            translation.getX(),
            translation.getY(),
            tag.getRotation()
        );
    }
    
    /**
     * get closest reef april tag pose to given position
     * 
     * @param pose field relative position
     * @return
     */
    public static Pose2d getClosestReefAprilTag(Pose2d pose) {
        return pose.nearest(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
            blueReefTagPoses :
            redReefTagPoses
        );

        // return reefTagPoses.stream().min((tagA, tagB) -> {
        //     double distA = tagA.getTranslation().getDistance(pose);
        //     double distB = tagB.getTranslation().getDistance(pose);
        //     return (int) Math.signum(distA - distB);
        // }).get();
    }

}
