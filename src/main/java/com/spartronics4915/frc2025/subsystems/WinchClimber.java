package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.Constants;
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.ClimberSpeeds;
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.WinchSpeeds;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kArmMotorConfig;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kArmMotorID;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kEngagedAngle;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kIntakeMotorConfig;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kIntakeMotorID;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kRetractedAngle;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kWinchMotorConfig;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kWinchMotorID;

import java.util.Set;

import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchClimber extends SubsystemBase implements ModeSwitchInterface {

    private BooleanPublisher isClimbedPublisher = NetworkTableInstance.getDefault().getTable("log").getBooleanTopic("is climbed").publish();
    private boolean isClimbed = false;
    private boolean isWinchEngaged = false;
    private final SparkBase mWinchMotor;
    private final SparkBase mArmMotor;
    private final SparkBase mIntakeMotor;
    private final RelativeEncoder mEncoder;

    private ClimberSpeeds operatorArmState = ClimberSpeeds.ENGAGE;
    private WinchSpeeds operatorWinchState = WinchSpeeds.RETRACT;

    public WinchClimber() {
        super();

        mWinchMotor = new SparkMax(kWinchMotorID, MotorType.kBrushless);
        mWinchMotor.configure(kWinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mArmMotor = new SparkMax(kArmMotorID, MotorType.kBrushless);
        mArmMotor.configure(kArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mIntakeMotor = new SparkMax(kIntakeMotorID, MotorType.kBrushless);
        mIntakeMotor.configure(kIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mEncoder = mArmMotor.getEncoder();    //figure out conversions

        isWinchEngaged = false;
        isClimbed = false;

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
        if (RobotBase.isSimulation()) return;
        SparkBaseConfig newConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        mArmMotor.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void turnArmBrakeModeOff() {
        if (RobotBase.isSimulation()) return;
        SparkBaseConfig newConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);

        mArmMotor.configureAsync(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command setArmCommand(double speed) {

        return this.runOnce(() -> {
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
            if (speed > 0) turnArmBrakeModeOff();
            setWinchSpeed(speed);
        });
    }

    public Command stopWinchCommand() {
        return this.runOnce(() -> {
            stopWinch();
            turnArmBrakeModeOn();
        });
    }

    public Command setClimberSpeedsCommand(ClimberSpeeds speed) {
        return setArmCommand(speed.speed);
    }

    public Command setWinchSpeedsCommand(WinchSpeeds speed) {
        return setWinchCommand(speed.speed);
    }

    public Command winchEngagedCommand() {
        return this.runOnce(() -> winchEngaged());
    }

    public Command operatorClimberArmCommand(boolean isPressed) {
        if (isPressed) return Commands.defer(() -> {
            return setClimberSpeedsCommand(operatorArmState);
        }, Set.of());
        else return Commands.runOnce(() -> {
            stopArm();
            switch (operatorArmState) {
                case ENGAGE: operatorArmState = ClimberSpeeds.RETRACT; break;
                case RETRACT: operatorArmState = ClimberSpeeds.ENGAGE; break;
            }
        });
    }

    public Command operatorClimberWinchCommand(boolean isPressed) {
        if (isPressed) return Commands.defer(() -> {
            return setWinchSpeedsCommand(operatorWinchState);
        }, Set.of());
        else return Commands.runOnce(() -> {
            stopWinch();
            turnArmBrakeModeOn();
            switch (operatorWinchState) {
                case EASE: operatorWinchState = WinchSpeeds.RETRACT; break;
                case RETRACT: operatorWinchState = WinchSpeeds.EASE; break;
            }
        });
    }

    // @Override
    // public void periodic() {
    // mMotor.set(mSpeedSetpoint);
    // }


    
    private boolean winchEngaged() {
        return kEngagedAngle <=mEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (kEngagedAngle <= mEncoder.getPosition()){
            isWinchEngaged = true;
        } else { isWinchEngaged = false; }

        if (kRetractedAngle >= mEncoder.getPosition()){
            isClimbed = true;
        } else { isClimbed = false; }

        if (isWinchEngaged==true) {
            mIntakeMotor.set(Constants.WinchClimberConstants.intakeSpeed);
        } else {
            mIntakeMotor.set(0.00);
        }

        SmartDashboard.putNumber("climberEncoder", mEncoder.getPosition());
    } 
    
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
