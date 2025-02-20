package com.spartronics4915.frc2025.commands.autos;

import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kPathConstraints;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kTagOffset;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchSide;
import com.spartronics4915.frc2025.commands.VariableAutos.ReefSide;
import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.AprilTagRegion;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AlignToReef {
    
    private final SwerveSubsystem mSwerve;

    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();


    public AlignToReef(SwerveSubsystem mSwerve, AprilTagFieldLayout field) {
        this.mSwerve = mSwerve;

        Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });
    }

    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired branch", Pose2d.struct).publish();

    public Command generateCommand(BranchSide side) {
        return Commands.defer(() -> {
            var branch = getClosestBranch(side, mSwerve);
            desiredBranchPublisher.accept(branch);
    
            return getPathFromWaypoint(getWaypointFromBranch(branch));
        }, Set.of());
    }


    public Command generateCommand(final ReefSide reefTag, BranchSide side) {
        return Commands.defer(() -> {
            var branch = getBranchFromTag(reefTag.getCurrent(), side);
            desiredBranchPublisher.accept(branch);
    
            return getPathFromWaypoint(getWaypointFromBranch(branch));
        }, Set.of());
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(mSwerve.getPose().getTranslation(), getPathVelocityHeading(mSwerve.getFieldVelocity(), waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return Commands.none();
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            kPathConstraints,
            new IdealStartingState(getVelocityMagnitude(mSwerve.getFieldVelocity()), mSwerve.getHeading()), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
    

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.05 ) {
            var diff =  mSwerve.getPose().minus(target).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle().rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    /**
     * 
     * @return Pathplanner waypoint with direction of travel away from the associated reef side
     */
    private Pose2d getWaypointFromBranch(Pose2d branch){
        return new Pose2d(
            branch.getTranslation(),
            branch.getRotation().rotateBy(Rotation2d.k180deg)
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
        
        return getBranchFromTag(tag, side);
    }


    private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side) {
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
        var alliance = DriverStation.getAlliance();
        
        ArrayList<Pose2d> reefPoseList;
        if (alliance.isEmpty()) {
            reefPoseList = allReefTagPoses;
        } else{
            reefPoseList = alliance.get() == Alliance.Blue ? 
                blueReefTagPoses :
                redReefTagPoses;
        }


        return pose.nearest(reefPoseList);

    }

}
