package com.spartronics4915.frc2025.commands.drive;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.commands.autos.AlignToReef;

import static com.spartronics4915.frc2025.Constants.DriveCommandConstants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.subsystems.vision.TargetDetectorInterface.Detection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;


public final class ChassisSpeedSuppliers {
    private static final PIDController mAnglePIDRad = new PIDController(kAnglePIDConstants.kP(), kAnglePIDConstants.kI(), kAnglePIDConstants.kD());
    
    static{
        mAnglePIDRad.enableContinuousInput(-Math.PI, Math.PI);
        RobotModeTriggers.teleop()
            .or(RobotModeTriggers.autonomous())
            .or(RobotModeTriggers.test())
            .onTrue(
                Commands.runOnce(() -> {
                    resetTeleopHeadingOffset();
                })
            );
    }

    public static boolean isFieldRelative = OI.kStartFieldRel;

    public static Rotation2d teleopHeadingOffset = Rotation2d.fromDegrees(0.0);

    public static LinearVelocity maxSpeed = Drive.kMaxSpeed.copy();
    public static AngularVelocity maxAngularVelocity = Drive.kMaxAngularSpeed.copy();


    /**
     * sets whether {@link #computeVelocitiesFromController computeVelocitiesFromController} is field or robot relative when not specified
     * @param newIsFieldRelative
     */
    public static void setFieldRelative(boolean newIsFieldRelative){
        if (newIsFieldRelative != isFieldRelative) {
            resetTeleopHeadingOffset();
        }
        isFieldRelative = newIsFieldRelative;
    }

    public static void resetMaxSpeed(){
        setMaxSpeed(Drive.kMaxSpeed, Drive.kMaxAngularSpeed);
    } 

    public static void setMaxSpeed(LinearVelocity linear, AngularVelocity angular){
        maxSpeed = linear;
        maxAngularVelocity = angular;
    }

    public static void setTeleopHeadingOffset(Rotation2d offset){
        teleopHeadingOffset = offset;
    }

    public static void resetTeleopHeadingOffset(){
        teleopHeadingOffset = shouldFlip() ? Rotation2d.fromDegrees(180) : new Rotation2d();
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

            // if (isFieldRelative) {
            //     var desiredAngle = getAngleJoystickAngle(driverController, swerve);
            //     cs.omegaRadiansPerSecond = gotoAngle(
            //         () -> desiredAngle, swerve
            //     ).get().omegaRadiansPerSecond;
            // }
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
            inputomegaraw = driverController.getRightX() * -1.0; // consider changing from angular velocity
    
            final double inputx = applyResponseCurve(MathUtil.applyDeadband(inputxraw, OI.kStickDeadband));
            final double inputy = applyResponseCurve(MathUtil.applyDeadband(inputyraw, OI.kStickDeadband));
            final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, OI.kStickDeadband));
    
            cs.vxMetersPerSecond = inputx * maxSpeed.in(MetersPerSecond);
            cs.vyMetersPerSecond = inputy * maxSpeed.in(MetersPerSecond);
            cs.omegaRadiansPerSecond = inputomega * maxAngularVelocity.in(RadiansPerSecond);

            if (isFieldRelative) {
                cs = ChassisSpeeds.fromFieldRelativeSpeeds(cs, swerve.getPose().getRotation());
                cs = rotateLinearChassisSpeeds(cs, teleopHeadingOffset);
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
            
            return new ChassisSpeeds(0, 0, inputomega * maxAngularVelocity.in(RadiansPerSecond));
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

    public static ChassisSpeeds rotateLinearChassisSpeeds(ChassisSpeeds in, Rotation2d offset){
        var modifiedLinear = new Translation2d(
            in.vxMetersPerSecond,
            in.vyMetersPerSecond
        ).rotateBy(offset);

        return new ChassisSpeeds(
            modifiedLinear.getX(),
            modifiedLinear.getY(), 
        in.omegaRadiansPerSecond);
    }

    /**
     * get field relative angle based on controller joysticks and alliance
     */
    public static Rotation2d getAngleJoystickAngle(XboxController driverController, SwerveSubsystem swerve){
        var rightX = driverController.getRightX() * -1.0;
        var rightY = driverController.getRightY() * -1.0;

        if (Math.hypot(rightY, rightX) < OI.kAngleStickDeadband) {
            return swerve.getPose().getRotation();
        }

        if (shouldFlip()) { //CHECKUP can be replaced by the teleop offset? but also messes with behavior
            rightX = -rightX;
            rightY = -rightY;
        }

        return new Rotation2d(rightY, rightX);
    }

    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    public static Rotation2d getFieldAngleBetween(Translation2d from, Translation2d to){
        return to.minus(from).getAngle();
    }

    public static Supplier<Rotation2d> orientTowardsReef(SwerveSubsystem swerve) {
        //return () -> Rotation2d.fromDegrees(45);
        return () -> {
            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                Pose2d reef = new Pose2d(4.5, 4, Rotation2d.kZero);
                if (Math.sqrt(Math.pow(reef.getX() - swerve.getPose().getX(), 2) + Math.pow(reef.getY() - swerve.getPose().getY(), 2)) < 6) 
                    return AlignToReef.getClosestReefAprilTag(swerve.getPose()).getRotation();
            }
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
                Pose2d reef = new Pose2d(13, 4, Rotation2d.kZero);
                if (Math.sqrt(Math.pow(reef.getX() - swerve.getPose().getX(), 2) + Math.pow(reef.getY() - swerve.getPose().getY(), 2)) < 6) 
                    return AlignToReef.getClosestReefAprilTag(swerve.getPose()).getRotation();
            }
            return swerve.getHeading();
        };
    }

    //#endregion


}
