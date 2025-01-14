package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    SparkMax motor1;
    //private SparkMax motor2;                  - this motor is a maybe
    
    public ClimberSubsystem () {                 //-need fill in motor Ids
        motor1 = new SparkMax(0, MotorType.kBrushless);
        //motor2 = new CANSparkMax(0, MotorType.kBrushless);
    }

    public void grab () {

    }

    public void moveUp () {

    }

    public void moveDown () {

    }

    public void manualControl (double newPosition) {

    }

    public double getClimberAngle () {
        return 0.0;
    }

    public void release () {
        /*
        while on cage, to get off the cage (gently)
        */
    }

}