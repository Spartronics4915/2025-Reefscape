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

            private double mCurrentSetPoint = 0.0;

                private void initPID() {
                PIDController mPidController = new PIDController(ArmConstants.kPIDConstants.kP, ArmConstants.kPIDConstants.kI, ArmConstants.kPIDConstants.kD);
                mPidController.setIZone(ArmConstants.kIZone);
            
            }

            public void setSetpoint(double newPOS) {
                //add saftey

                mCurrentSetPoint = newPOS;
            }

            @Override
            public void periodic() {
                //add trapazoid motion profile

               // setVoltage(
                    //mPidController.calculate(getPosition(), mCurrentSetPoint)
                //);
            }

            
            
        

    public void moveToIntake() {
        
    }


    public void moveToScore() {

    }

    public void moveToStow() {

    }

}
