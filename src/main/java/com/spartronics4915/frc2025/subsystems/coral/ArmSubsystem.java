package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase { 
    
    SparkMax mMotor = new SparkMax(1, MotorType.kBrushless);

    public void moveToIntake() {
        
        var mEncoder = mMotor.getEncoder();
            //mEncoder.setPositionConversionFactor(kPositionConversionFactor);

    }

    public void moveToScore() {

    }

}
