package com.spartronics4915.frc2025.commands.autos;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LineFollower extends Command {

    private final SwerveSubsystem swerve;
    private final Field2d field;
    private final Pose2d targetPose;
    private final TrapezoidProfile trapezoidProfile;
    private final TrapezoidProfile.Constraints motionConstraints;
    private double velocitySetpoint;

    private final double kFinalSpeed = 0.5;
    private final double kMaxVelocity = 1;
    private final double kAcceleration = 0.5;
    private final double kStartingFollowDistance = 1;

    private double currFollowDistance;

    public LineFollower(SwerveSubsystem swerve, Pose2d targetPose) {

        this.swerve = swerve;
        this.targetPose = targetPose;

        field = new Field2d();
        SmartDashboard.putData("LineFollower", field);
        motionConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kAcceleration);
        trapezoidProfile = new TrapezoidProfile(motionConstraints);

        currFollowDistance = kStartingFollowDistance;

        addRequirements(swerve);
    }

    public static Translation2d getClosestPointToLine(Translation2d robotPosition, Pose2d lineOrigin) {

        Vector<N2> lineVec = VecBuilder.fill(lineOrigin.getRotation().getCos(), lineOrigin.getRotation().getSin());
        Vector<N2> botVec = robotPosition.minus(lineOrigin.getTranslation()).toVector();
        var projectionVec = botVec.projection(lineVec);
        Translation2d closestPoint = new Translation2d(projectionVec).plus(lineOrigin.getTranslation());
        return closestPoint;

    }

    // The follower point is the point chosen ahead of the robot that the robot is following
    
    public static Translation2d chooseFollowerPoint(Pose2d currPose, double lookaheadDistance) {

        Translation2d followerOffset = new Translation2d(lookaheadDistance,0).rotateBy(currPose.getRotation());

        return followerOffset.plus(currPose.getTranslation());
    }
    // For the Pose2d this assumes that the pose is looking at the robot
    public static Translation2d chooseFollowerTargetPoint(Translation2d currentPosition, Pose2d targetPose,
            double lookaheadDistance) {

            Translation2d closestLinePoint = getClosestPointToLine(currentPosition, targetPose);

        return Translation2d.kZero;
    }

    public static double estimateDistanceToTarget(Translation2d robotPoint, Translation2d followerTargetPoint,
            Translation2d targetPoint) {

        return followerTargetPoint.getDistance(robotPoint) + targetPoint.getDistance(followerTargetPoint);
    }

    public static double updateLookaheadDistance(Translation2d robotPoint, Translation2d targetPoint) {

    }
    public double chooseVelocity(Translation2d robotPoint, Translation2d followerPoint,
            Translation2d targetPoint) {
            
                double distanceToTarget = estimateDistanceToTarget(robotPoint, followerPoint, targetPoint);
                TrapezoidProfile.State goalState = new TrapezoidProfile.State(distanceToTarget, kFinalSpeed);
                TrapezoidProfile.State currState = new TrapezoidProfile.State(0, velocitySetpoint);
                TrapezoidProfile.State outputState = trapezoidProfile.calculate(distanceToTarget, currState, goalState);

                return outputState.velocity;
    }

    @Override
    public void initialize() {

        velocitySetpoint = swerve.getRobotVelocity().vxMetersPerSecond;
        Pose2d currPose = swerve.getPose();
        Translation2d closestPoint = getClosestPointToLine(currPose.getTranslation(), targetPose);
        Translation2d followerPoint = chooseFollowerPoint(currPose, 1);
        field.getObject("ClosestPoint").setPose(new Pose2d(closestPoint, targetPose.getRotation()));
        field.setRobotPose(currPose);
        System.out.println(currPose);
        System.out.println(followerPoint);
    }

}
