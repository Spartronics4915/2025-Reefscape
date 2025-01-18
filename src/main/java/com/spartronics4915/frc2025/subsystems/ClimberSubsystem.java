package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax motor1;
    private SparkMax motor2;
    
    //boolean isInverted = motor2.configAccessor.getInverted();

    public enum ClimberState {
        CAGE(Rotation2d.fromDegrees(0), 7),
        STOW(Rotation2d.fromDegrees(0), 5),;
        
        
        public final Rotation2d angle;
        public final double claw;

        private ClimberState(Rotation2d angle, double claw) {
                    this.angle = angle;
                    this.claw = claw;
        }

        public Rotation2d getAngle() {
            return angle;
        }
        

    }
    
    public ClimberSubsystem () {                 //-need fill in motor Ids
        motor1 = new SparkMax(0, MotorType.kBrushless);
        motor2 = new SparkMax(0, MotorType.kBrushless);
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