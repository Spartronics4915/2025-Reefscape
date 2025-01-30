package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    //private SparkMax motorLeft;
    private SparkMax motorRight;
    
    private PIDController mPidController;

    public enum ClimberState {
        
        LIFTED(Rotation2d.fromDegrees(Constants.ClimberConstants.liftedAngle)),
        STOW(Rotation2d.fromDegrees(Constants.ClimberConstants.stowAngle)),;
        
        public final Rotation2d angle;

        private ClimberState(Rotation2d angle) {
            this.angle = angle;
            
        }

        

    }
    
    public ClimberSubsystem () {                 //-need fill in motor Ids
        /*
        SparkMaxConfig Left = new SparkMaxConfig();
        Left.idleMode(IdleMode.kBrake);
        motorLeft = new SparkMax(1, MotorType.kBrushless);
        motorLeft.configure(Left, null, null);
        */


        SparkMaxConfig Right = new SparkMaxConfig();
        Right.idleMode(IdleMode.kBrake);
        motorRight = new SparkMax(0, MotorType.kBrushless);
        motorRight.configure(Right, null, null);


    }

    public void setposition () {

    }

    public void moveUp () {

    }

    public void moveDown () {

    }

    public void manualControl (double newPosition) {

    }

    public double getClimberAngle () {
        double position = motorRight.getEncoder().getPosition();
        return position;

    }

    private void initiPID() {
        //mPidController = new PIDController(kPIDConstants.kP(), kPIDConstants.kI(), kPIDConstants.kD());
        mPidController = new PIDController(0,0,0);
        mPidController.setIZone(kIZone);



    }

    public void setSetpoint(double newPos){



        mPidController.setSetpoint(newPos);
    }

    @Override
    public void periodic() {
        setVoltage(
            mPidController.calculate(getPosition)
        );
    }
}