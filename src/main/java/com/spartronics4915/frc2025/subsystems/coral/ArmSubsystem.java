package com.spartronics4915.frc2025.subsystems.coral;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.spartronics4915.frc2025.Constants.ArmConstants.ArmSubsystemState;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class ArmSubsystem extends SubsystemBase implements ModeSwitchInterface{ 
    
    private TalonFX mArmMotor;
    private ArmFeedforward mFFCalculator;

    private TrapezoidProfile mArmProfile;

    private Rotation2d mCurrentSetPoint = Rotation2d.fromRotations(0);;
    private State mCurrentState;

    public ArmSubsystem() {
        
        mFFCalculator = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV,ArmConstants.kS);

        initArmMotor();
        
        initArmProfile();

        resetMechanism();
    }

    private void initArmMotor() {
        mArmMotor = new TalonFX(ArmConstants.kArmMotorID);
        var mConfigurator = mArmMotor.getConfigurator();
        mConfigurator.apply(ArmConstants.kPIDConfigs);
    }

    public void resetMechanism(){
        var position = getPosition();
        mCurrentSetPoint = (position);
        mCurrentState = new State(angleToRaw(position), 0.0);
    }

    private Rotation2d convertRaw(double rotation) {
        Rotation2d angle = Rotation2d.fromDegrees(rotation);
        return angle;
    }

    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    private Rotation2d getPosition() {
        var position = mArmMotor.getRotorPosition().getValueAsDouble();

        return convertRaw(position);
    }

    private void initArmProfile() {
        mArmProfile = new TrapezoidProfile(ArmConstants.kConstraints);
        mCurrentState = new State(angleToRaw(getPosition()), 0.0);
    }

    @Override
    public void periodic() {
        
        //need set points as a imput
        mCurrentSetPoint = Rotation2d.fromRotations(
            MathUtil.clamp(mCurrentSetPoint.getRotations(), ArmConstants.kMinAngle.getRotations(), ArmConstants.kMaxAngle.getRotations()));

        mCurrentState = mArmProfile.calculate(ArmConstants.kDt, mCurrentState, new State((angleToRaw(mCurrentSetPoint)), 0.0));

        final PositionVoltage m_request = new PositionVoltage(mCurrentState.position).withSlot(0).withFeedForward(mFFCalculator.calculate(mCurrentState.position, mCurrentState.velocity));
        
        mArmMotor.setControl(m_request);


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