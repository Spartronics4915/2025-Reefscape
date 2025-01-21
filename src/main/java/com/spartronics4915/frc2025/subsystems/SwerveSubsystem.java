package com.spartronics4915.frc2025.subsystems;

import java.io.File;
import java.io.IOException;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase implements ModeSwitchInterface{


    private final SwerveDrive swerveDrive;

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("pose", Pose2d.struct).publish();

    public SwerveSubsystem(SwerveDirectories swerveDir) {

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), swerveDir.directory)).createSwerveDrive(Drive.kMaxSpeed,
            // new Pose2d(new Translation2d(Meter.of(2),
            //     Meter.of(5)),
            //     Rotation2d.fromDegrees(180)
            // )
                guessStartingPosition()
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        Shuffleboard.getTab("logging").addNumber("yaw", () -> getPose().getRotation().getDegrees());
        Shuffleboard.getTab("logging").addNumber("x", () -> getPose().getX());
        Shuffleboard.getTab("logging").addNumber("y", () -> getPose().getY());


        swerveDrive.setMotorIdleMode(true);
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        // swerveDrive.resetOdometry(new Pose2d(1.5, 5, Rotation2d.fromDegrees(45)));

        // NetworkTableInstance.getDefault().getTable("swerveLogging").getStructArrayTopic("modules", SwerveModulePosition.struct)
        // Shuffleboard.getTab("swerveLogging").add

        // AutoBuilder.configure(
        //     this::getPose, 
        //     swerveDrive::resetOdometry, 
        //     swerveDrive::getRobotVelocity, 
        //     (speeds, FF) -> drive(speeds), 
        //     new PPHolonomicDriveController(
        //         Drive.AutoConstants.kTranslationPID, 
        //         Drive.AutoConstants.kRotationPID), 
        //     Drive.AutoConstants.kRobotConfig, 
        //     () -> {
        //         Optional<Alliance> temp = DriverStation.getAlliance();
        //         if(temp.isEmpty()) return false;
        //         if (temp.get() == Alliance.Red) {return true;}
        //         return false;
        //     }, this);

    }

    private static Pose2d guessStartingPosition() {

        if  (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {

            return new Pose2d(8,6, Rotation2d.fromDegrees(0));
        }
        else {

            return new Pose2d(10,2, Rotation2d.fromDegrees(90));

        }

    }

    // External API for sending explicit driving commands to the swerve drive
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public void driveFieldOriented(ChassisSpeeds chassisSpeeds) {
        swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setPose(Pose2d pose) {
        swerveDrive.swerveDrivePoseEstimator.resetPose(pose);
        if(RobotBase.isSimulation()) {
            swerveDrive.getMapleSimDrive().get().setSimulationWorldPose(pose);
        }
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public void stopChassis() {
        drive(new ChassisSpeeds());
    }

    public Command stopChassisCommand() {
        return Commands.runOnce(() -> stopChassis(), this);
    }

    public SwerveDrive getInternalSwerve() {
        return swerveDrive;
    }

    @Override
    public void onDisable() {
        swerveDrive.setMotorIdleMode(false);
    }

    @Override
    public void onModeSwitch() {
        swerveDrive.setMotorIdleMode(true);
    }

    @Override
    public void periodic() {
        posePublisher.accept(getPose());
    }

}
