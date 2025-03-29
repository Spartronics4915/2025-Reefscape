package com.spartronics4915.frc2025.subsystems.coral;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
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
import com.spartronics4915.frc2025.util.CoralSim;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface{
    
    private SparkMax mMotor1;
    private SparkClosedLoopController closedLoopController;

    public double setpoint = 0.0; 

    // private var sensor;
    private LaserCan lc;
    private LaserCan pipeLC;

    private final DoublePublisher appliedOutPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("applied out").publish();
    private final DoublePublisher velocityPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("Velocity").publish();
    private final BooleanPublisher lCPub = NetworkTableInstance.getDefault().getTable("logIntake").getBooleanTopic("LC").publish();
    private final DoublePublisher pipeDistPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("Pipe LC Dist").publish();
    private final BooleanPublisher l4pipePub = NetworkTableInstance.getDefault().getTable("logIntake").getBooleanTopic("L4 PipeLC").publish();

    private Debouncer l4Debouncer = new Debouncer(kBranchLCDebounceTime);

    private RelativeEncoder mEncoder;


    public IntakeSubsystem() {
        // mMotor1 = new SparkMax(IntakeConstants.kMotorID1, MotorType.kBrushless);
        mMotor1 = new SparkMax(kMotorID, MotorType.kBrushless);

        //mMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mMotor1.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoopController = mMotor1.getClosedLoopController();
        
        lc = new LaserCan(kLaserCANID);
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }

        pipeLC = new LaserCan(kPipeLCID);
        try {
            pipeLC.setRangingMode(LaserCan.RangingMode.SHORT);
            pipeLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            pipeLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        mEncoder = mMotor1.getEncoder();

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

        setpoint = newSpeed;
    }

    private void setPercentage(double newPercentage) {
        setpoint = newPercentage;
        mMotor1.set(newPercentage);
    }

// Not sure if it works with being void, when it outputs if something is detected.
    // public void detect() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
     
    //     SmartDashboard.putBoolean("LaserCanDetect", measurement.distance_mm<=laserCANDistance);
    // }

    public boolean detect(){
        if (RobotBase.isSimulation()) {
            return CoralSim.getIntakeLC();
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

    public boolean branchLC(){
        return branchLCCache;
    }

    private boolean branchLCCache = false;

    @Override
    public void periodic() {
        appliedOutPub.accept(mMotor1.getAppliedOutput());
        velocityPub.accept(mEncoder.getVelocity());
        lCPub.accept(detect());

        var measure = pipeLC.getMeasurement();
        if (measure == null) {
            pipeDistPub.accept(-1.0);
            branchLCCache = l4Debouncer.calculate(false);
            l4pipePub.accept(branchLCCache);;
        } else{
            pipeDistPub.accept(measure.distance_mm);
            branchLCCache = l4Debouncer.calculate(measure.distance_mm < kBranchLCTriggerDist);
            l4pipePub.accept(branchLCCache);
        }
    }

    @Override
    public void onModeSwitch() {
        intakeMotors(IntakeSpeed.NEUTRAL);
    }
}