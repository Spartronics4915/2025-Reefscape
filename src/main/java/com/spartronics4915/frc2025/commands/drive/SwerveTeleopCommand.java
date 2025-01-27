package com.spartronics4915.frc2025.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.getSwerveTeleopCSSupplier;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;

    private static final StructPublisher<Rotation2d> desiredAnglePub = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("desired Angle", Rotation2d.struct).publish();

    public SwerveTeleopCommand(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

        this.driverController = driverController;

        setFieldRelative(true);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        ChassisSpeeds cs = getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem).get();

        swerveSubsystem.drive(cs);
    }

    public void setHeadingOffset(Rotation2d offset) {
        ChassisSpeedSuppliers.setTeleopHeadingOffset(offset);
    }

    public void resetHeadingOffset() {
        ChassisSpeedSuppliers.resetTeleopHeadingOffset();
    }

    public void setFieldRelative(boolean fieldRelative) {
        ChassisSpeedSuppliers.setFieldRelative(fieldRelative);
    }

    public boolean getFieldRelative() {
        return ChassisSpeedSuppliers.isFieldRelative;
    }
}
