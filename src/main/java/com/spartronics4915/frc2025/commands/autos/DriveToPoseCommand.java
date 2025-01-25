package com.spartronics4915.frc2025.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class DriveToPoseCommand extends Command {

    private Pose2d targetPose;
    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;
    private final SwerveSubsystem swerve;
    private final double closeEnoughThreshold;
    private double velocitySetpoint;
    private double rotationVelocitySetpoint;
    private boolean nearGoal; 
    private final double minimumSpeed;

    private Supplier<Pose2d> poseSupplier;
    /*
     * Command that uses a trapezoidal profile to drive to a point in a smooth
     * fashion.
     */

    public DriveToPoseCommand(Pose2d targetPose, Constraints translationContraints, 
    Constraints rotationConstraints,
            double closeEnoughThreshold, double minimumSpeed,
            SwerveSubsystem swerve) {
        this.targetPose = targetPose;
        this.translationProfile = new TrapezoidProfile(translationContraints);
        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
        this.swerve = swerve;
        this.closeEnoughThreshold = closeEnoughThreshold;
        this.minimumSpeed = minimumSpeed;

        poseSupplier = null;
        velocitySetpoint = 0;
        nearGoal = false;

        addRequirements(swerve);
    }

    public DriveToPoseCommand(Supplier<Pose2d> targetPoseSupplier, Constraints translationContraints, 
    Constraints rotationConstraints,
            double closeEnoughThreshold, double minimumSpeed,
            SwerveSubsystem swerve) {
        this.targetPose = null;
        this.translationProfile = new TrapezoidProfile(translationContraints);
        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
        this.swerve = swerve;
        this.closeEnoughThreshold = closeEnoughThreshold;
        this.minimumSpeed = minimumSpeed;

        poseSupplier = targetPoseSupplier;
        velocitySetpoint = 0;
        nearGoal = false;

        addRequirements(swerve);
    }


    @Override
    public void initialize() {

        velocitySetpoint = 0;
        nearGoal = false;
        if(targetPose == null) {
            targetPose = poseSupplier.get();
        }
    }

    @Override
    public void execute() {

        Pose2d currPose = swerve.getPose();
        Translation2d targetPosition = targetPose.getTranslation();
        Translation2d translationToTarget = targetPosition.minus(currPose.getTranslation());
        double distanceRemaining = translationToTarget.getNorm();
        //System.out.println(currPose + " " + targetPoint + " " + translationToTarget + " " + distanceRemaining);

        if((distanceRemaining < closeEnoughThreshold) || nearGoal) {

            nearGoal = true;
            return;
        }
        // ChassisSpeeds currChassisSpeeds = swerve.getFieldVelocity();
        // double currVelocity = Math.hypot(currChassisSpeeds.vxMetersPerSecond,
        // currChassisSpeeds.vyMetersPerSecond);

        final double dT = 1 / 50.;

        // Compute the velocities for translation
        State goalState = new State(distanceRemaining, 0);
        State currentState = new State(0, velocitySetpoint);
        State outputState = translationProfile.calculate(dT, currentState, goalState);

        double driveVelocity = outputState.velocity;

        // Compute the velocity for rotation

        Rotation2d rotationRemaining = targetPose.getRotation().minus(currPose.getRotation());
        goalState = new State(rotationRemaining.getRadians(), 0);
        currentState = new State()

        if(driveVelocity < minimumSpeed) {
            driveVelocity = minimumSpeed;
        }

        Translation2d driveTranslation = translationToTarget.div(translationToTarget.getNorm())
                .times(driveVelocity);

        ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(driveTranslation.getX(),
                driveTranslation.getY(), 0);

        swerve.driveFieldOriented(newChassisSpeeds);
        velocitySetpoint = outputState.velocity;
    }

    @Override
    public boolean isFinished() {

        return nearGoal;
    }

}
