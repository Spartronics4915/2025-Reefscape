package com.spartronics4915.frc2025.commands.autos;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final double kMaxVelocity = 5;
    private final double kAcceleration = 3;
    private final double kStartingFollowDistance = 1;
    private final double dT = 1./50;

    private final PIDController pidController;
    private final double kP = 54;
    private final double maxRotVelocity = Math.PI*4;

    private double currFollowDistance;
    public double lookaheadDistance;

    private boolean isFinished;

    public LineFollower(SwerveSubsystem swerve, Pose2d targetPose, double lookaheadDistance) {

        this.swerve = swerve;
        this.targetPose = targetPose;

        field = new Field2d();
        SmartDashboard.putData("LineFollower", field);
        motionConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kAcceleration);
        trapezoidProfile = new TrapezoidProfile(motionConstraints);

        this.lookaheadDistance = lookaheadDistance;
        this.pidController = new PIDController(kP, 0, 0);

        currFollowDistance = kStartingFollowDistance;
        isFinished = false;

        addRequirements(swerve);
    }

    public static Translation2d getClosestPointToLine(Translation2d robotPosition, Pose2d lineOrigin) {

        Vector<N2> lineVec = VecBuilder.fill(lineOrigin.getRotation().getCos(), lineOrigin.getRotation().getSin());
        Vector<N2> botVec = robotPosition.minus(lineOrigin.getTranslation()).toVector();
        var projectionVec = botVec.projection(lineVec);
        Translation2d closestPoint = new Translation2d(projectionVec).plus(lineOrigin.getTranslation());
        return closestPoint;

    }

    // The follower point is the point chosen ahead of the robot that the robot is
    // following

    public static Translation2d chooseFollowerPoint(Pose2d currPose, double lookaheadDistance) {

        Translation2d followerOffset = new Translation2d(lookaheadDistance, 0).rotateBy(currPose.getRotation());

        return followerOffset.plus(currPose.getTranslation());
    }

    // For the Pose2d this assumes that the pose is looking at the robot
    public static Translation2d getVectorToFollowerPoint(Translation2d currentPosition, Pose2d targetPose,
            double lookaheadDistance) {

        Translation2d closestLinePoint = getClosestPointToLine(currentPosition, targetPose);

        double closestLineDistance = closestLinePoint.getDistance(currentPosition);
        Translation2d closestPtVec = closestLinePoint.minus(currentPosition);
        Translation2d vecToTarget = targetPose.getTranslation().minus(currentPosition);
        if (closestLineDistance > lookaheadDistance) {

            return closestPtVec;
        } // Make a constant

        double remainderDist = Math
                .sqrt(lookaheadDistance * lookaheadDistance - closestLineDistance * closestLineDistance);

        Translation2d vecToFollower = new Translation2d(Math.signum(vecToTarget.getX()) * Math.abs(closestPtVec.getY()),
                Math.signum(vecToTarget.getY()) * Math.abs(closestPtVec.getX()));

        vecToFollower = vecToFollower.times(remainderDist / vecToFollower.getNorm()).plus(closestPtVec);
        return vecToFollower;
    }

    public static double estimateDistanceToTarget(Translation2d robotPoint, Translation2d followerTargetPoint,
            Translation2d targetPoint) {

        return followerTargetPoint.getDistance(robotPoint) + targetPoint.getDistance(followerTargetPoint);
    }

    public double updateLookaheadDistance(Translation2d robotPoint, Translation2d targetPoint) {

        double dist = robotPoint.getDistance(targetPoint);

        if (dist < lookaheadDistance) {

            return lookaheadDistance / 2;
        } else
            return lookaheadDistance;
    }

    public double chooseVelocity(Translation2d robotPoint, Translation2d followerPoint,
            Translation2d targetPoint) {

        double distanceToTarget = estimateDistanceToTarget(robotPoint, followerPoint, targetPoint);
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(distanceToTarget, kFinalSpeed);
        TrapezoidProfile.State currState = new TrapezoidProfile.State(0, velocitySetpoint);
        TrapezoidProfile.State outputState = trapezoidProfile.calculate(dT, currState, goalState);

        return outputState.velocity;
    }

    @Override
    public void initialize() {

        velocitySetpoint = swerve.getRobotVelocity().vxMetersPerSecond;
        field.getObject("TargetPose").setPose(targetPose);

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void execute() {

        Pose2d currPose = swerve.getPose();
        Translation2d closestPoint = getClosestPointToLine(currPose.getTranslation(), targetPose);
        //field.getObject("ClosestPoint").setPose(new Pose2d(closestPoint, targetPose.getRotation()));
        field.setRobotPose(currPose);

        lookaheadDistance = updateLookaheadDistance(currPose.getTranslation(), targetPose.getTranslation());
        if (lookaheadDistance < 0.25) {
            isFinished = true;
        }

        Translation2d vecToFollower = getVectorToFollowerPoint(currPose.getTranslation(), targetPose, lookaheadDistance);
        Translation2d followerPoint = currPose.getTranslation().plus(vecToFollower);
        velocitySetpoint = chooseVelocity(currPose.getTranslation(), followerPoint, targetPose.getTranslation());

        Rotation2d vecToFollowerAngle = vecToFollower.getAngle();
        Rotation2d tx = currPose.getRotation().minus(vecToFollowerAngle);
        double angleVelocity = pidController.calculate(tx.getRadians(), 0);

        angleVelocity = MathUtil.clamp(angleVelocity, -this.maxRotVelocity, this.maxRotVelocity);


        field.getObject("Follower").setPose(new Pose2d(followerPoint, targetPose.getRotation().rotateBy(Rotation2d.k180deg)));
        swerve.drive(new ChassisSpeeds(velocitySetpoint, 0, angleVelocity));

    }

}
