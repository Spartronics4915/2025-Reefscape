package com.spartronics4915.frc2025.commands.autos;

import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kAutoAlignAdjustTimeout;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kAutoAlignPredict;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kAutoPathConstraints;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kTeleopAlignAdjustTimeout;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kTeleopPathConstraints;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchSide;
import com.spartronics4915.frc2025.commands.VariableAutos.ReefSide;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.AprilTagRegion;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

    static{
        var field = RobotContainer.getFieldLayout();

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

    public static void warmup(){
        System.out.println("auto align is warmed up: \nBlue: " + blueReefTagPoses.hashCode() +"\nRed: " + redReefTagPoses.hashCode() + "\nBoth: " + allReefTagPoses.hashCode());
    }

    public boolean isPIDLoopRunning = false;


    public AlignToReef(SwerveSubsystem mSwerve) {
        this.mSwerve = mSwerve;

    }

    /**
     * this is an enum that represents if the branch is on the left or right side ofthe field, instead of relative to the tag
     */
    public enum FieldBranchSide{
        LEFT(BranchSide.LEFT),
        RIGHT(BranchSide.RIGHT),
        MIDDLE(BranchSide.MIDDLE);

        public BranchSide branchSide;

        public FieldBranchSide getOpposite(){
            switch (this){
                case LEFT: return FieldBranchSide.RIGHT;
                case RIGHT: return FieldBranchSide.LEFT;
                case MIDDLE: return FieldBranchSide.MIDDLE;
            }
            System.out.println("Error, switch case failed to catch the field branch side");
            return this;
        }

        private FieldBranchSide(BranchSide internal) {
            this.branchSide = internal;
        }
    }

    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired branch", Pose2d.struct).publish();

    private PathConstraints pathConstraints = kAutoPathConstraints;

    public void changePathConstraints(PathConstraints newPathConstraints){
        this.pathConstraints = newPathConstraints;
    }

    public Command generateCommand(FieldBranchSide side) {
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
        }, Set.of(mSwerve));
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(mSwerve.getPose().getTranslation(), getPathVelocityHeading(mSwerve.getFieldVelocity(), waypoint)),
            waypoint
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return 
            Commands.sequence(
                Commands.print("start position PID loop"),
                PositionPIDCommand.generateCommand(mSwerve, waypoint, kAutoAlignAdjustTimeout),
                Commands.print("end position PID loop")
            );
        }
        var startingVel = getVelocityMagnitude(mSwerve.getFieldVelocity());

        if (DriverStation.isAutonomous()) {
            startingVel = MetersPerSecond.of(
                Math.max(startingVel.in(MetersPerSecond), 0.1)
            );
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            DriverStation.isAutonomous() ? pathConstraints : kTeleopPathConstraints,
            new IdealStartingState(
                startingVel,
                mSwerve.getHeading()
            ), 
            new GoalEndState(0.0, waypoint.getRotation())
        );

        path.preventFlipping = true;

        return (AutoBuilder.followPath(path).andThen(
            Commands.print("start position PID loop"),
            PositionPIDCommand.generateCommand(mSwerve, waypoint, (
                DriverStation.isAutonomous() ? kAutoAlignAdjustTimeout : kTeleopAlignAdjustTimeout
            ))
                .beforeStarting(Commands.runOnce(() -> {isPIDLoopRunning = true;}))
                .finallyDo(() -> {isPIDLoopRunning = false;}),
            Commands.print("end position PID loop")
        )).finallyDo((interupt) -> {
            if (interupt) { //if this is false then the position pid would've X braked & called the same method
                mSwerve.drive(new ChassisSpeeds(0,0,0));
            }
        });
    }
    

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            System.out.println("approach: straight line");
            var diff = target.getTranslation().minus(mSwerve.getPose().getTranslation());
            System.out.println("diff calc: \nx: " + diff.getX() + "\ny: " + diff.getY() + "\nDoT: " + diff.getAngle().getDegrees());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }

        System.out.println("approach: compensating for velocity");

        var rotation = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        
        System.out.println("velocity calc: \nx: " + cs.vxMetersPerSecond + "\ny: " + cs.vyMetersPerSecond + "\nDoT: " + rotation);

        return rotation;
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

    public static Pose2d getClosestBranch(FieldBranchSide fieldSide, SwerveSubsystem swerve){
        Pose2d swervePose = swerve.predict(kAutoAlignPredict);
        
        Pose2d tag = getClosestReefAprilTag(swervePose);
        
        BranchSide tagSide = fieldSide.branchSide;

        if (
            swervePose.getX() > 4.500
            &&
            swervePose.getX() < 13
        ) {
            tagSide = fieldSide.getOpposite().branchSide;
        }

        return getBranchFromTag(tag, tagSide);
    }


    private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side) {
        var translation = tag.getTranslation().plus(
            new Translation2d(
                side.tagOffset.getY(),
                side.tagOffset.getX()
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
