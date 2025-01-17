package com.spartronics4915.frc2025.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class DriveToPointCommand extends Command {

    private final Translation2d targetPoint;
    private final TrapezoidProfile translationProfile;
    private final SwerveSubsystem swerve;
    private final double closeEnoughThreshold;
    private double velocitySetpoint;
    private boolean nearGoal; 
    private final double minimumSpeed;

    /*
     * Command that uses a trapezoidal profile to drive to a point in a smooth
     * fashion.
     */

    public DriveToPointCommand(Translation2d targetPoint, Constraints translationContraints,
            double closeEnoughThreshold, double minimumSpeed,
            SwerveSubsystem swerve) {
        this.targetPoint = targetPoint;
        this.translationProfile = new TrapezoidProfile(translationContraints);
        this.swerve = swerve;
        this.closeEnoughThreshold = closeEnoughThreshold;
        this.minimumSpeed = minimumSpeed;

        velocitySetpoint = 0;
        nearGoal = false;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        velocitySetpoint = 0;
        nearGoal = false;
    }

    @Override
    public void execute() {

        Pose2d currPose = swerve.getPose();
        Translation2d translationToTarget = targetPoint.minus(currPose.getTranslation());
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

        State goalState = new State(distanceRemaining, 0);
        State currentState = new State(0, velocitySetpoint);
        State outputState = translationProfile.calculate(dT, currentState, goalState);

        double driveVelocity = outputState.velocity;

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
