package com.spartronics4915.frc2025.subsystems.coral;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.spartronics4915.frc2025.Constants.IntakeConstants;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;

import edu.wpi.first.wpilibj.Filesystem;

public class IntakeSubsystem {
    
    private SparkMax mMotor1;
    private SparkMax mMotor2;

    private var sensor;

    private static IntakeSubsystem mInstance = null;

    private static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }

    public IntakeSubsystem() {
        mMotor1 = new SparkMax(IntakeConstants.kMotorID1, MotorType.kBrushless);
        mMotor2 = new SparkMax(IntakeConstants.kMotorID2, MotorType.kBrushless);
        sensor = new SparkMax(IntakeConstants.kSensorID);
    }

    public boolean detect () {
        /*activate and return value */
    }

    public void in () {
        /*activate motor */
    }

    public void out () {
        /*activate motor in reverse */
    }
}
