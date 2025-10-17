package com.spartronics4915.frc2025.subsystems;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.ClimberSpeeds;
import com.spartronics4915.frc2025.Constants.WinchClimberConstants.WinchSpeeds;

import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kArmMotorConfig;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kArmMotorID;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kEngagedAngle;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kEngageTarget;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kRetractTarget;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kRetractedAngle;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kWinchMotorConfig;
import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.kWinchMotorID;
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
    private boolean disableIntake = false;
    private final SparkBase mWinchMotor;
    private final SparkBase mArmMotor;
    private final RelativeEncoder mEncoder;
    private final RelativeEncoder mWinchEncoder;

    private double initialWinchPosition;

    private boolean invertedControls = false;

    public WinchClimber() {
        super();

        mWinchMotor = new SparkMax(kWinchMotorID, MotorType.kBrushless);
        mWinchMotor.configure(kWinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mArmMotor = new SparkMax(kArmMotorID, MotorType.kBrushless);
        mArmMotor.configure(kArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mWinchEncoder = mWinchMotor.getEncoder();
        initialWinchPosition= mWinchEncoder.getPosition();

        mEncoder = mArmMotor.getEncoder();    //figure out conversions

        mEncoder.setPosition(0.0);
        
        // mEncoder.setPosition(kStartingAngle.getRotations());

        mWinchMotor.set(0.0);
        mArmMotor.set(0.0);

        SmartDashboard.putData("ClimberUnspool", unSpoolWinch());
        SmartDashboard.putData("ClimberEngage", engageCommand());
        SmartDashboard.putData("ClimberRetract", retractCommand());
        
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

        return Commands.runOnce(() -> {
            setArmSpeed(speed);
        });
    }

    public Command stopArmCommand() {

        return Commands.runOnce(() -> {
            stopArm();
        });
    }

    public Command setWinchCommand(double speed) {
        return Commands.runOnce(() -> {
            if (speed > 0) turnArmBrakeModeOff();
            setWinchSpeed(speed);
        });
    }

    public Command stopWinchCommand() {
        return Commands.runOnce(() -> {
            stopWinch();
            turnArmBrakeModeOn();
        });
    }

    public Command unSpoolWinch(){
        return Commands.sequence(
            setWinchSpeedsCommand(WinchSpeeds.EASE),
            Commands.waitUntil(() -> {
                return mWinchEncoder.getPosition() > initialWinchPosition;
            }),
            stopWinchCommand()
        );
    }

    public Command setClimberSpeedsCommand(ClimberSpeeds speed) {
        return setArmCommand(speed.speed);
    }

    public Command setWinchSpeedsCommand(WinchSpeeds speed) {
        return setWinchCommand(speed.speed);
    }

    public Command winchEngagedCommand() {
        return Commands.runOnce(() -> winchEngaged());
    }

    public Command operatorClimberArmCommand(boolean isPressed) {
        if (isPressed) return Commands.defer(() -> {
            return setClimberSpeedsCommand(invertedControls ? ClimberSpeeds.RETRACT : ClimberSpeeds.ENGAGE);
        }, Set.of());
        else return Commands.runOnce(() -> {
            stopArm();
            turnArmBrakeModeOn();
        });
    }

    public Command operatorClimberWinchCommand(boolean isPressed) {
        if (isPressed) return Commands.defer(() -> {
            return setWinchSpeedsCommand(invertedControls ? WinchSpeeds.RETRACT : WinchSpeeds.EASE);
        }, Set.of());
        else return Commands.runOnce(() -> {
            stopWinch();
            turnArmBrakeModeOn();
        });
    }

    public Command invertOperatorClimberControls() {
        return Commands.runOnce(() -> invertedControls = !invertedControls);
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
        if (mEncoder.getPosition() <= kEngagedAngle){
            isWinchEngaged = true;
        } else { isWinchEngaged = false; }

        if (kRetractedAngle >= mEncoder.getPosition()){
            isClimbed = true;
        } else { isClimbed = false; }

        SmartDashboard.putNumber("climberEncoder", Math.floor(mEncoder.getPosition() * 1000) / 1000);
        SmartDashboard.putNumber("winchEncoder", mWinchEncoder.getPosition());
    } 
    
    @Override
    public void onModeSwitch() {
        stopWinch();
        stopArm();
        //disableIntake = true; 
    }

    @Override
    public void onDisable() {
        stopWinch();
        stopArm();
        disableIntake = true;
    }

    public Command engageCommand(){
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() ->
                    mEncoder.getPosition() < kEngageTarget
                ),
                setWinchSpeedsCommand(WinchSpeeds.EASE),
                setClimberSpeedsCommand(ClimberSpeeds.ENGAGE)
            ),
            stopArmCommand(),
            stopArmCommand()
        ).finallyDo(() -> {
            stopArm();
            stopWinch();
        });
    }

    public Command retractCommand(){
        return Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() ->
                    mEncoder.getPosition() > kRetractTarget
                ),
                stopArmCommand(),
                setWinchSpeedsCommand(WinchSpeeds.RETRACT)
            ),
            stopWinchCommand(),
            stopArmCommand()
        ).finallyDo(() -> {
            stopArm();
            stopWinch();
        });
    }

}
