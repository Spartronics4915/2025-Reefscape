package com.spartronics4915.frc2025.commands.driveCommands;

import java.util.Optional;
import java.util.function.Supplier;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import static com.spartronics4915.frc2025.Constants.DriveCommandConstants.*;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface.Detection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class ChassisSpeedSuppliers {
    private static final PIDController mAnglePID = new PIDController(kAnglePIDConstants.kP(), kAnglePIDConstants.kI(), kAnglePIDConstants.kD());


    
    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    private static void invertBasedOnAlliance(ChassisSpeeds cs){
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
            cs.vyMetersPerSecond = -cs.vyMetersPerSecond;
        }
    }

    public static Supplier<ChassisSpeeds> computeRotationalVelocityFromController(XboxController driverController, SwerveSubsystem swerve){
        return () -> {
            final double inputomegaraw;
            if (RobotBase.isSimulation()) {
                inputomegaraw = driverController.getRawAxis(3) * -1.0;
            } else {
                inputomegaraw = driverController.getRightY() * 1.0; // consider changing from angular velocity
                // control to direct angle control
            }
            
            final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, OI.kStickDeadband));
            
            return new ChassisSpeeds(0, 0, inputomega * Drive.kMaxAngularSpeed);
        };
    }

    /**
     * @return the robot relative chassis speeds
     */
    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, boolean isFieldRelative, SwerveSubsystem swerve) {

        var internalSwerve = swerve.getInternalSwerve();
        var swerveController = swerve.getInternalSwerve().getSwerveController();
        

        return () -> {
            var cs = swerveController.getTargetSpeeds(
                -driverController.getLeftY(), 
                -driverController.getLeftX(), 
                driverController.getRightX(), 
                driverController.getRightX(), 
                internalSwerve.getOdometryHeading().getRadians(), 
                Drive.kMaxSpeed
            );

            if (isFieldRelative) {
                invertBasedOnAlliance(cs); 
                cs = ChassisSpeeds.fromRobotRelativeSpeeds(cs, swerve.getPose().getRotation());
            }
            return cs;
        };

        // return () -> {

        //     ChassisSpeeds cs = new ChassisSpeeds();
    
        //     // Need to verify that we are using the right axes.
        //     final double inputxraw = driverController.getLeftY() * -1.0;
        //     final double inputyraw = driverController.getLeftX() * -1.0;
        //     final double inputomegaraw;
        //     if (RobotBase.isSimulation()) {
        //         inputomegaraw = driverController.getRawAxis(3) * -1.0;
        //     } else {
        //         inputomegaraw = driverController.getRightY() * -1.0; // consider changing from angular velocity
        //         // control to direct angle control
        //     }
    
        //     final double inputx = applyResponseCurve(MathUtil.applyDeadband(inputxraw, OI.kStickDeadband));
        //     final double inputy = applyResponseCurve(MathUtil.applyDeadband(inputyraw, OI.kStickDeadband));
        //     final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, OI.kStickDeadband));
    
        //     cs.vxMetersPerSecond = inputx * Drive.kMaxSpeed;
        //     cs.vyMetersPerSecond = inputy * Drive.kMaxSpeed;
        //     cs.omegaRadiansPerSecond = inputomega * Drive.kMaxAngularSpeed;
    
        //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && isFieldRelative) {
        //         cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
        //         cs.vyMetersPerSecond = -cs.vyMetersPerSecond;
        //     }
    
        //     if (isFieldRelative) {
        //         cs = ChassisSpeeds.fromFieldRelativeSpeeds(cs, swerve.getPose().getRotation());
        //     }
    
        //     return cs;
        // };
    }

    public static Supplier<ChassisSpeeds> gotoAngle(Supplier<Rotation2d> fieldRelativeAngleSupplier, SwerveSubsystem mSwerve){
        return () -> {
            return new ChassisSpeeds(0, 0,
                mAnglePID.calculate(fieldRelativeAngleSupplier.get().getRotations(), mSwerve.getPose().getRotation().getRotations())
            );
        };
    }

    public static Supplier<ChassisSpeeds> targetDetector(Supplier<Optional<Detection>> detectionSupplier, double maxRotVelocity){
        return () -> {
            Optional<Detection> detection = detectionSupplier.get();

            // Set the rotational velocity to zero because it will be replaced later.
            // If there is no detection, it will drive forward
            ChassisSpeeds cs = new ChassisSpeeds();

            if(detection.isPresent()) {

                double tx;

                tx = detection.get().tx();
                double angleVelocity = mAnglePID.calculate(tx, 0);
                angleVelocity = MathUtil.clamp(angleVelocity, -maxRotVelocity, maxRotVelocity);
                SmartDashboard.putNumber("angleVelocity", angleVelocity);
                double inputRotVelocity = angleVelocity / 180 * Math.PI;
                cs.omegaRadiansPerSecond = inputRotVelocity;
            }
            return cs;
        };
    }

}
