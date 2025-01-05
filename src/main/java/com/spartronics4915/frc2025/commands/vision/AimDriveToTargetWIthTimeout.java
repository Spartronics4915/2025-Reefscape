package com.spartronics4915.frc2025.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface.Detection;

public class AimDriveToTargetWIthTimeout extends Command {

    private final TargetDetectorInterface targetDetector;
    private final double timeoutThreshold;
    private Timer timeoutTimer;
    private final SwerveSubsystem swerveSubsystem;
    private boolean finished;
    private Rotation2d lastHeadingSetpoint;
    private final PIDController pidController;
    private final double maxRotVelocity;

    // The default behavior is that the command will keep driving towards the last
    // detected location
    // if the detection is lost, until the timeout. At the timeout, it just exits,
    // no new drive commands
    // are issued. It is up to the following command to issue the next step to the
    // robot.

    // TODO: Add a max velocity magnitude to clamp.

    public AimDriveToTargetWIthTimeout(TargetDetectorInterface targetDetector, SwerveSubsystem swerveSubsystem,
            double kP, double timeoutThreshold, double maxRotVelocity) {

        this.targetDetector = targetDetector;
        this.timeoutThreshold = timeoutThreshold;
        this.swerveSubsystem = swerveSubsystem;
        this.pidController = new PIDController(kP, 0, 0);
        this.maxRotVelocity = maxRotVelocity;

        timeoutTimer = new Timer();
        finished = false;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        timeoutTimer.stop();
        timeoutTimer.reset();

        lastHeadingSetpoint = null;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void execute() {

        Optional<Detection> detection = targetDetector.getClosestVisibleTarget();
        if (lastHeadingSetpoint == null) {
            lastHeadingSetpoint = swerveSubsystem.getPose().getRotation();
        }

        Rotation2d currHeadingSetpoint;
        double tx;
        if (finished) {
            return;
        }

        if (detection.isEmpty()) {

            timeoutTimer.start();
            // If the timer is running and has passed the threshold, return.
            if (timeoutTimer.hasElapsed(timeoutThreshold)) {
                finished = true;
                return;
            } else {
                // Otherwise, don't update the setpoint, that way the motion continues smoothly
                // until we
                // reacquire

                currHeadingSetpoint = lastHeadingSetpoint;
                tx = currHeadingSetpoint.minus(swerveSubsystem.getPose().getRotation()).getDegrees();
            }
        } else {

            tx = detection.get().tx();

            currHeadingSetpoint = swerveSubsystem.getPose().getRotation().plus(Rotation2d.fromDegrees(tx));
            lastHeadingSetpoint = currHeadingSetpoint;

            double angleVelocity = pidController.calculate(tx, 0);

            angleVelocity = MathUtil.clamp(angleVelocity, -this.maxRotVelocity, this.maxRotVelocity);

            double inputAngleVelocity = angleVelocity / 180 * Math.PI;

            swerveSubsystem.drive(new ChassisSpeeds(computeForwardVelocity(), 0, inputAngleVelocity));

            if (timeoutTimer.isRunning()) {
                // If it is not empty, we can reset the timeout timer. Because we got a
                // detection
                timeoutTimer.stop();
                timeoutTimer.reset();
            }
        }

    }

    public double computeForwardVelocity() {
        return 1;
    }
}
