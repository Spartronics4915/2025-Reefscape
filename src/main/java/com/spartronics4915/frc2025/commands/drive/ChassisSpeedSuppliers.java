package com.spartronics4915.frc2025.commands.drive;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class ChassisSpeedSuppliers {
    private static final PIDController mAnglePIDRad = new PIDController(kAnglePIDConstants.kP(), kAnglePIDConstants.kI(), kAnglePIDConstants.kD());
    static{
        mAnglePIDRad.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static boolean isFieldRelative = OI.kStartFieldRel;

    /**
     * sets whether {@link #computeVelocitiesFromController computeVelocitiesFromController} is field or robot relative when not specified
     * @param newIsFieldRelative
     */
    public static void setFieldRelative(boolean newIsFieldRelative){
        isFieldRelative = newIsFieldRelative;
    }

    //#region linear and rotation CS suppliers

    /**
     * this returns a supplier of chassis speeds for all teleoperated control.
     * 
     * this can be used in conjunction with the RotationIndependentControlCommand class to smoothly switch between automated and driver control
     */
    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(XboxController driverController, SwerveSubsystem swerve){
        return () -> {
            ChassisSpeeds cs = computeVelocitiesFromController(driverController, swerve).get();

            if (isFieldRelative) {
                var desiredAngle = getAngleJoystickAngle(driverController, swerve);
                cs.omegaRadiansPerSecond = gotoAngle(
                    () -> desiredAngle, swerve
                ).get().omegaRadiansPerSecond;
            }
            return cs;
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, SwerveSubsystem swerve) {
        return computeVelocitiesFromController(driverController, isFieldRelative, swerve);
    }

        /**
     * @return the robot relative chassis speeds
     */
    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, boolean isFieldRelative, SwerveSubsystem swerve) {

        return () -> {

            ChassisSpeeds cs = new ChassisSpeeds();
    
            // Need to verify that we are using the right axes.
            final double inputxraw = driverController.getLeftY() * -1.0;
            final double inputyraw = driverController.getLeftX() * -1.0;
            final double inputomegaraw;
            if (RobotBase.isSimulation()) {
                inputomegaraw = driverController.getRawAxis(3) * -1.0;
            } else {
                inputomegaraw = driverController.getRightX() * -1.0; // consider changing from angular velocity
                // control to direct angle control
            }
    
            final double inputx = applyResponseCurve(MathUtil.applyDeadband(inputxraw, OI.kStickDeadband));
            final double inputy = applyResponseCurve(MathUtil.applyDeadband(inputyraw, OI.kStickDeadband));
            final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, OI.kStickDeadband));
    
            cs.vxMetersPerSecond = inputx * Drive.kMaxSpeed;
            cs.vyMetersPerSecond = inputy * Drive.kMaxSpeed;
            cs.omegaRadiansPerSecond = inputomega * Drive.kMaxAngularSpeed;
    
            //inverts based on alliance
            //CHECKUP need to make sure this works
            if (shouldFlip() && isFieldRelative) {
                cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
                cs.vyMetersPerSecond = -cs.vyMetersPerSecond;
            }
    
            if (isFieldRelative) {
                cs = ChassisSpeeds.fromFieldRelativeSpeeds(cs, swerve.getPose().getRotation());
            }
    
            return cs;
        };
    }


    //#endregion
    //#region rotational CS suppliers

    public static Supplier<ChassisSpeeds> controllerRotationVelocity(XboxController driverController, SwerveSubsystem swerve){
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

    
    public static Supplier<ChassisSpeeds> gotoAngle(Supplier<Rotation2d> fieldRelativeAngleSupplier, SwerveSubsystem mSwerve){
        return () -> {
            return new ChassisSpeeds(0, 0,
                mAnglePIDRad.calculate(mSwerve.getPose().getRotation().getRadians(), fieldRelativeAngleSupplier.get().getRadians())
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
                double angleVelocity = mAnglePIDRad.calculate(tx, 0);
                angleVelocity = MathUtil.clamp(angleVelocity, -maxRotVelocity, maxRotVelocity);
                SmartDashboard.putNumber("angleVelocity", angleVelocity);
                double inputRotVelocity = angleVelocity / 180 * Math.PI;
                cs.omegaRadiansPerSecond = inputRotVelocity;
            }
            return cs;
        };
    }


    //#endregion 

    //#region utility

    public static boolean shouldFlip(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }
    
    /**
     * get field relative angle based on controller joysticks and alliance
     */
    public static Rotation2d getAngleJoystickAngle(XboxController driverController, SwerveSubsystem swerve){
        var rightX = driverController.getRightX();
        var rightY = driverController.getRightY();

        if (Math.hypot(rightY, rightX) < OI.kAngleStickDeadband) {
            return swerve.getPose().getRotation();
        }

        if (shouldFlip()) {
            rightX = -rightX;
            rightY = -rightY;
        }

        return new Rotation2d(rightY, rightX);
    }

    private static void invertBasedOnAlliance(ChassisSpeeds cs){
        if (shouldFlip()) {
            cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
            cs.vyMetersPerSecond = -cs.vyMetersPerSecond;
        }
    }

    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    //#endregion


}
