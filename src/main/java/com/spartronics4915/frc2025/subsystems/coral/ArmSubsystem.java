package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase { 
    
    SparkMax mMotor = new SparkMax(1, MotorType.kBrushless);
    RelativeEncoder mEncoder = mMotor.getEncoder();
    
    public ArmSubsystem() {
        
    }
    
    public void moveToIntake() {
        
    }

    public void moveToScore() {

    }

    public void moveToStow() {

    }
}
