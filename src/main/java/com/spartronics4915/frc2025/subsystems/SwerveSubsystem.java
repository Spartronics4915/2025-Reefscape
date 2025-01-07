package com.spartronics4915.frc2025.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;

import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {

    private static SwerveSubsystem mInstance = null;

    public static SwerveSubsystem getInstance(){
        if (mInstance == null) {
            mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),SwerveDirectories.NEO.directory));
        }
        return mInstance;
    }

    private final SwerveDrive swerveDrive;

    private SwerveSubsystem(File directory) {

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Drive.kMaxSpeed,
                new Pose2d(new Translation2d(Meter.of(2),
                    Meter.of(5)),
                    Rotation2d.fromDegrees(180)
                )
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        swerveDrive.resetOdometry(new Pose2d(1.5, 5, Rotation2d.fromDegrees(45)));
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

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public void stopChassis() {
        drive(new ChassisSpeeds());
    }

    public Command stopChassisCommand() {
        return Commands.runOnce(() -> stopChassis(), this);
    }

}
