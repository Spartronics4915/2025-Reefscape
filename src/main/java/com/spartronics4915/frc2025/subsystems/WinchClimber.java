package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchClimber extends SubsystemBase implements ModeSwitchInterface {

    private final SparkBase mWinchMotor;
    private final SparkBase mArmMotor;
    // private final RelativeEncoder mEncoder;

    public WinchClimber() {
        super();

        mWinchMotor = new SparkMax(kWinchMotorID, MotorType.kBrushless);
        mWinchMotor.configure(kWinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mArmMotor = new SparkMax(kArmMotorID, MotorType.kBrushless);
        mArmMotor.configure(kArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // mEncoder = mMotor.getEncoder();

        // mEncoder.setPosition(kStartingAngle.getRotations());

        mWinchMotor.set(0.0);
        mArmMotor.set(0.0);
    }

    public void setArmSpeed(double speed) {

        mArmMotor.set(speed);
    }

    public void setWinchSpeed(double speed) {
        mWinchMotor.set(speed);
    }

    public void stopWinch() {
        mWinchMotor.set(0.0);
    }

    public void stopArm() {
        mArmMotor.set(0);
    }

    private void turnArmBrakeModeOn() {
        SparkBaseConfig newConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        mArmMotor.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void turnArmBrakeModeOff() {
        SparkBaseConfig newConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);

        mArmMotor.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command setArmCommand(double speed) {

        return this.runOnce(() -> {
            System.out.println("Arm " +speed);

            setArmSpeed(speed);
        });
    }

    public Command stopArmCommand() {

        return this.runOnce(() -> {
            stopArm();
        });
    }

    public Command setWinchCommand(double speed) {
        return this.runOnce(() -> {
            System.out.println("winch " + speed);

            if (speed > 0) {
                turnArmBrakeModeOff();

            }
            setWinchSpeed(speed);
        });
    }

    public Command stopWinchCommand() {
        return this.runOnce(() -> {
            stopWinch();
            turnArmBrakeModeOn();
        });
    }

    // @Override
    // public void periodic() {
    // mMotor.set(mSpeedSetpoint);
    // }

    @Override
    public void onModeSwitch() {
        stopWinch();
        stopArm();
    }

    @Override
    public void onDisable() {
        stopWinch();
        stopArm();
    }

}
