package com.spartronics4915.frc2025.subsystems.coral;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import static com.spartronics4915.frc2025.Constants.IntakeConstants.*;
import static edu.wpi.first.units.Units.RPM;

import com.spartronics4915.frc2025.Constants.IntakeConstants;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface{
    
    private SparkMax mMotor1;
    private SparkClosedLoopController closedLoopController;

    // private var sensor;
    private LaserCan lc;

    public IntakeSubsystem() {
        // mMotor1 = new SparkMax(IntakeConstants.kMotorID1, MotorType.kBrushless);
        mMotor1 = new SparkMax(kMotorID, MotorType.kBrushless);

        //mMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mMotor1.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoopController = mMotor1.getClosedLoopController();
        
        lc = new LaserCan(kLaserCANID);
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            // lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }

        Shuffleboard.getTab("loggingIntake").addDouble("appliedOut", mMotor1::getAppliedOutput);
        Shuffleboard.getTab("loggingIntake").addDouble("velocity", ()-> mMotor1.getEncoder().getVelocity());
        Shuffleboard.getTab("loggingIntake").addBoolean("laserCAN", ()-> {
            
            LaserCan.Measurement measurement = lc.getMeasurement();
            if (measurement == null) {
                return false;
            }

            return measurement.distance_mm < IntakeConstants.laserCANDistance;
        });



        SmartDashboard.putData("IntakeSpeed: IN", setPresetSpeedCommand(IntakeSpeed.IN));
        SmartDashboard.putData("IntakeSpeed: NEUTRAL", setPresetSpeedCommand(IntakeSpeed.NEUTRAL));
        SmartDashboard.putData("IntakeSpeed: OUT", setPresetSpeedCommand(IntakeSpeed.OUT));

        var lcTrigger = new Trigger(() -> detect()).debounce(kLaserCanDebounce).onTrue(setPresetSpeedCommand(IntakeSpeed.NEUTRAL));

    }

    private void setSpeed(double newSpeed) {
        closedLoopController.setReference(
            newSpeed,
            ControlType.kVelocity
        );
    }

    private void setPercentage(double newPercentage) {
        mMotor1.set(newPercentage);
    }

// Not sure if it works with being void, when it outputs if something is detected.
    // public void detect() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
     
    //     SmartDashboard.putBoolean("LaserCanDetect", measurement.distance_mm<=laserCANDistance);
    // }

    public boolean detect(){
        if (RobotBase.isSimulation()) {
            return true;
        }

        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement == null) {
            return false;
        }

        return measurement.distance_mm < IntakeConstants.laserCANDistance;
    }

    public void intakeMotors (IntakeSpeed preset) {
        // setSpeed(preset.intakeSpeed);
        setPercentage(preset.intakePercentage);
    }

    public Command setSpeedCommand(double newSpeed){
        return Commands.runOnce(() -> setSpeed(newSpeed));
    }

    public Command setPresetSpeedCommand(IntakeSpeed preset){
        return Commands.runOnce(() -> intakeMotors(preset));
    }

    public AngularVelocity getSpeed(){
        return RPM.of(mMotor1.getEncoder().getVelocity());
    }
    // @Override
    // public void periodic() {
    //     detect();
    // }

    @Override
    public void onModeSwitch() {
        intakeMotors(IntakeSpeed.NEUTRAL);
    }
}