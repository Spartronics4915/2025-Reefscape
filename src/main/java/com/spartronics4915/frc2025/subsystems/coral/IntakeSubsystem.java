package com.spartronics4915.frc2025.subsystems.coral;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
// import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.spartronics4915.frc2025.Constants.IntakeConstants;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem {
    
    private SparkMax mMotor1;
    private SparkClosedLoopController closedLoopController;

    // private var sensor;
    LaserCan lc = new LaserCan(0);

    public IntakeSubsystem() {
        // mMotor1 = new SparkMax(IntakeConstants.kMotorID1, MotorType.kBrushless);
        mMotor1 = new SparkMax(1, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        closedLoopController = mMotor1.getClosedLoopController();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);

        //mMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }
    }

    private void setSpeed(double newSpeed) {
        closedLoopController.setReference(
            newSpeed,
            ControlType.kVelocity
        );
    }

// Not sure if it works with being void, when it outputs if something is detected.
    public void detect() {
        LaserCan.Measurement measurement = lc.getMeasurement();
     
        SmartDashboard.putBoolean("LaserCanDetect", measurement.distance_mm<=IntakeConstants.laserCANDistance);
    }

    public void intakeMotors (IntakeSpeed preset) {
        setSpeed(preset.intakeSpeed);
    }
}