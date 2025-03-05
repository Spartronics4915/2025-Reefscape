package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchClimber extends SubsystemBase implements ModeSwitchInterface{
    
    private final SparkBase mMotor;
    private final RelativeEncoder mEncoder;

    private double mSpeedSetpoint = 0.0;

    public WinchClimber() {
        super();

        mMotor = new SparkMax(kMotorID, MotorType.kBrushless);
        mMotor.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mEncoder = mMotor.getEncoder();

        mEncoder.setPosition(kStartingAngle.getRotations());

        mMotor.set(0.0);
    }

    public void setWinchSpeed(double speed){
        mSpeedSetpoint = speed;
    }

    public void stopWinch(){
        mMotor.set(0.0);
        mSpeedSetpoint = 0.0;
    }

    public Command driveWinch(double speed){
        return this.startEnd(() -> {
            setWinchSpeed(speed);
        }, () -> {
            stopWinch();
        });
    }

    public Command setWinchCommand(double speed){
        return this.runOnce(() -> {
            setWinchSpeed(speed);
        });
    }

    public Command stopWinchCommand(){
        return this.runOnce(() -> {
            stopWinch();
        });
    }

    @Override
    public void periodic() {
        mMotor.set(mSpeedSetpoint);
    }

    @Override public void onModeSwitch() {stopWinch();}
    @Override public void onDisable() {stopWinch();}

}
