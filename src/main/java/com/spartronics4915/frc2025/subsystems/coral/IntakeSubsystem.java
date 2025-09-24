package com.spartronics4915.frc2025.subsystems.coral;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static com.spartronics4915.frc2025.Constants.IntakeConstants.*;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.spartronics4915.frc2025.Constants.IntakeConstants;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.util.CoralSim;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface{
    
    private TalonFX mMotor1;
    private VelocityVoltage mVelocityVoltage = new VelocityVoltage(0);

    public double setpoint = 0.0; 

    // private var sensor;
    private LaserCan lc;
    private LaserCan pipeLC;

    private final DoublePublisher appliedOutPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("applied out").publish();
    private final DoublePublisher velocityPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("Velocity").publish();
    private final BooleanPublisher lCPub = NetworkTableInstance.getDefault().getTable("logIntake").getBooleanTopic("LC").publish();
    private final DoublePublisher pipeDistPub = NetworkTableInstance.getDefault().getTable("logIntake").getDoubleTopic("Pipe LC Dist").publish();
    private final BooleanPublisher l4pipePub = NetworkTableInstance.getDefault().getTable("logIntake").getBooleanTopic("L4 PipeLC").publish();
    private final BooleanPublisher l4RawpipePub = NetworkTableInstance.getDefault().getTable("logIntake").getBooleanTopic("L4 PipeLC Raw").publish();

    private Debouncer l4Debouncer = new Debouncer(kBranchLCDebounceTime);

    public IntakeSubsystem() {
        mMotor1 = new TalonFX(kMotorID);

        mMotor1.setNeutralMode(NeutralModeValue.Brake);
        var mConfigurator = mMotor1.getConfigurator();
        mConfigurator.apply(kPIDConfigs);
        mConfigurator.apply(kCurrentLimits);
        mConfigurator.apply(kFeedbackConfig);
        mConfigurator.apply(motorOutputConfigs);

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

        SmartDashboard.putData("IntakeSpeed: IN", setPresetSpeedCommand(IntakeSpeed.IN));
        SmartDashboard.putData("IntakeSpeed: NEUTRAL", setPresetSpeedCommand(IntakeSpeed.NEUTRAL));
        SmartDashboard.putData("IntakeSpeed: OUT", setPresetSpeedCommand(IntakeSpeed.OUT));

        var lcTrigger = new Trigger(() -> detect()).debounce(kLaserCanDebounce).onTrue(setPresetSpeedCommand(IntakeSpeed.NEUTRAL));

    }

    private void setSpeed(double newSpeed) {
        mVelocityVoltage.Velocity = newSpeed;
        mMotor1.setControl(mVelocityVoltage);

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
        return RPM.of(mMotor1.getVelocity().getValue().in(RPM));
    }

    public boolean branchLC(){
        return updateCache();
    }

    private boolean branchLCCache = false;

    public boolean updateCache(){
        var measure = pipeLC.getMeasurement();
        if (measure == null) {
            pipeDistPub.accept(-1.0);
            branchLCCache = l4Debouncer.calculate(false);
            l4pipePub.accept(branchLCCache);;
        } else{
            l4RawpipePub.accept(measure.distance_mm < kBranchLCTriggerDist);
            pipeDistPub.accept(measure.distance_mm);
            branchLCCache = measure.distance_mm < kBranchLCTriggerDist;
            l4pipePub.accept(branchLCCache);
        }
        return branchLCCache;
    }

    @Override
    public void periodic() {
        appliedOutPub.accept(mMotor1.getMotorVoltage().getValue().in(Volts));
        velocityPub.accept(mMotor1.getVelocity().getValue().in(RPM));
        lCPub.accept(detect());

        updateCache();
    }

    @Override
    public void onModeSwitch() {
        intakeMotors(IntakeSpeed.NEUTRAL);
    }
}