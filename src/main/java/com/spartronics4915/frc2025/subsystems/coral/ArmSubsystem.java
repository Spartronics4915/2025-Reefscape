package com.spartronics4915.frc2025.subsystems.coral;


import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.spartronics4915.frc2025.Constants.ArmConstants.ArmSubsystemState;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class ArmSubsystem extends SubsystemBase implements ModeSwitchInterface{ 
    
    private TalonFX mArmMotor;
    private ArmFeedforward mFFCalculator;

    private TrapezoidProfile mArmProfile;

    private Rotation2d mCurrentSetPoint = Rotation2d.fromRotations(0);;
    private State mCurrentState;

    private final DoublePublisher appliedOutPub = NetworkTableInstance.getDefault().getTable("log").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPub = NetworkTableInstance.getDefault().getTable("log").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePub = NetworkTableInstance.getDefault().getTable("log").getStructTopic("desiredState", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointpub = NetworkTableInstance.getDefault().getTable("log").getStructTopic("setpointpub", Rotation2d.struct).publish();


    public ArmSubsystem() {
        
        mFFCalculator = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV,ArmConstants.kS);

        initArmMotor();
        
        initArmProfile();

        setMechanismAngle(Rotation2d.kZero);
    
        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    private void initArmMotor() {
        mArmMotor = new TalonFX(ArmConstants.kArmMotorID);
        var mConfigurator = mArmMotor.getConfigurator();
        mConfigurator.apply(ArmConstants.kPIDConfigs);
    }

    public void resetMechanism(){
        var position = getPosition();
        mCurrentSetPoint = position;
        mCurrentState = new State(angleToRaw(position), 0.0);
    }

    private Rotation2d rawToAngle(double rotation) {
        Rotation2d angle = Rotation2d.fromRotations(rotation);
        return angle;
    }

    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    private Rotation2d getPosition() {
        var position = mArmMotor.getRotorPosition().getValue().in(Rotations);

        return rawToAngle(position);
    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(angleToRaw(getPosition()), 0.0);
    }

    @Override
    public void periodic() {

        //need set points as a imput
        mCurrentSetPoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                mCurrentSetPoint.getRotations(), 
                ArmConstants.kMinAngle.getRotations(), 
                ArmConstants.kMaxAngle.getRotations()
        ));

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State((angleToRaw(mCurrentSetPoint)), 0.0));

        final PositionVoltage m_request = new PositionVoltage(mCurrentState.position).withFeedForward(mFFCalculator.calculate(mCurrentState.position, mCurrentState.velocity));
        
        mArmMotor.setControl(m_request);

        updateUserOuputs();
    }

    private void updateUserOuputs() {
        appliedOutPub.accept(mArmMotor.getMotorVoltage().getValue().in(Volts));
        positionPub.accept(getPosition());
        desiredStatePub.accept(rawToAngle(mCurrentState.position));
        setpointpub.accept(mCurrentSetPoint);
    }
    
    private void setMechanismAngle(Rotation2d angle){
        mArmMotor.setPosition(angleToRaw(angle));
        resetMechanism();
    }

    public void setSetpoint(Rotation2d newSetpoint){
        mCurrentSetPoint = newSetpoint;
    }

    public void setSetpoint(ArmSubsystemState newState) {
        setSetpoint(newState.angle);
    }

    public void incrementAngle(Rotation2d delta){
        mCurrentSetPoint = mCurrentSetPoint.plus(delta);
    }


    //Commands 

    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(ArmSubsystemState preset){
        return setSetpointCommand(preset.angle);
    }

    public Command setMechanismAngleCommand(Rotation2d newAngle){
        return this.runOnce(() -> setMechanismAngle(newAngle));
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }
}