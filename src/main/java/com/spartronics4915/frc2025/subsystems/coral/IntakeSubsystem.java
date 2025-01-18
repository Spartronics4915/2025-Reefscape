package com.spartronics4915.frc2025.subsystems.coral;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.spartronics4915.frc2025.Constants.IntakeConstants;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;

import edu.wpi.first.wpilibj.Filesystem;

public class IntakeSubsystem {
    
    private SparkMax mMotor1;
    private SparkMax mMotor2;

    private var sensor;

    public IntakeSubsystem() {
        mMotor1 = new SparkMax(IntakeConstants.kMotorID1, MotorType.kBrushless);
        mMotor2 = new SparkMax(IntakeConstants.kMotorID2, MotorType.kBrushless);
        sensor = new SparkMax(IntakeConstants.kSensorID);
    }

    public boolean detect () {
        /*activate and return value */
    }

    public void intakeMotors (IntakeSpeed preset) {
        mMotor1.set(preset.intakeSpeed);
        mMotor2.set(preset.intakeSpeed);
    }
}