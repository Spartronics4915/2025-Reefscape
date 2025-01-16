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
    
        /*public ArmSubsystem() {
        mEncoder.setPositionConversionFactor(ArmConstants.kPositionConversionFactor);
        }*/

        private double getPosition() {
            return mEncoder.getPosition();
        }

        /*private void initPID() {
        PIDController mPIDController = mMotor.getPIDController();
        mPIDController.setP()
        mPIDController.setI()
        mPIDController.setD()
        }*/

        public class WPILibProfileSubsystem {
            private TrapazoidProfile mProfile;
            private PIDController mPidController;

            public WPILibProfileSubsystem() {
                super();

                //initPID();
                //initProfile();
            }
            
        }

        public class TrapazoidProfile {
            private double mCurrentSetPoint = 0.2;
            
        }
        
            
        

    public void moveToIntake() {
        
    }


    public void moveToScore() {

    }

    public void moveToStow() {

    }

}
