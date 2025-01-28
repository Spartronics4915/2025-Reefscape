package com.spartronics4915.frc2025.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MotorSimulationSubsystem extends SubsystemBase{
    public enum MechanismSims{
        // 0 = straight down
        // intake side swinging = positive
        ARM(new PhysicalSimWrapper(
            new SingleJointedArmSim(
                DCMotor.getFalcon500(1),
                25.7143,
                0.1395451863,
                Units.inchesToMeters(17.429886),
                0,
                2*Math.PI,
                true,
                0
            )
        )),
        ELEVATOR(new PhysicalSimWrapper(
            new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                    DCMotor.getNEO(2),
                    Units.lbsToKilograms(9),
                    Units.inchesToMeters(1.63800),
                    25
                ),
                DCMotor.getNEO(2),
                Units.inchesToMeters(36.000000),
                Units.inchesToMeters(88.000000),
                true,
                Units.inchesToMeters(36.000000)
            )
        )),
        GRABBER_FLYWHEEL(new PhysicalSimWrapper(
            new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getNeo550(1), 
                    0.00003068 * 2.0, 
                    4.0
                ),
                DCMotor.getNeo550(1)
            )
        ));

        public PhysicalSimWrapper sim;

        private MechanismSims(PhysicalSimWrapper sim) {
            this.sim = sim;
        }
    }


    public static class PhysicalSimWrapper{
        private DCMotorSim mDCMotorSim = null;
        private FlywheelSim mFlywheelSim = null;
        private SingleJointedArmSim mSingleJointedArmSim = null;
        private ElevatorSim mElevatorSim = null;
        
        @SafeVarargs
        private <T> T getNonNull(T... items){
            for (T t : items) {
                if (t != null) {
                    return t;
                } 
            }
            throw new NullPointerException("only null values provided");
        }

        public PhysicalSimWrapper(DCMotorSim mDCMotorSim) {this.mDCMotorSim = mDCMotorSim;}
        public PhysicalSimWrapper(FlywheelSim mFlywheelSim) {this.mFlywheelSim = mFlywheelSim;}
        public PhysicalSimWrapper(SingleJointedArmSim mSingleJointedArmSim) {this.mSingleJointedArmSim = mSingleJointedArmSim;}
        public PhysicalSimWrapper(ElevatorSim mElevatorSim) {this.mElevatorSim = mElevatorSim;}

        public LinearSystemSim<?,?,?> getSim(){
            return getNonNull(mDCMotorSim, mFlywheelSim, mSingleJointedArmSim, mElevatorSim);
        }

        public Double getCurrentDrawAmps(){
            if(mDCMotorSim != null) return mDCMotorSim.getCurrentDrawAmps();
            else if(mFlywheelSim != null) return mFlywheelSim.getCurrentDrawAmps();
            else if(mSingleJointedArmSim != null) return mSingleJointedArmSim.getCurrentDrawAmps();
            else if(mElevatorSim != null) return mElevatorSim.getCurrentDrawAmps();
            else throw new NullPointerException("only null sims provided");
        }

        /**
         * 
         * @return depending on the sim, flywheel = null, arm / DC = angle in rad, elevator = height in meters
         */
        public Double getPosition(){
            if(mDCMotorSim != null) return mDCMotorSim.getAngularPositionRad();
            else if(mFlywheelSim != null) return null;
            else if(mSingleJointedArmSim != null) return mSingleJointedArmSim.getAngleRads();
            else if(mElevatorSim != null) return mElevatorSim.getPositionMeters();
            else throw new NullPointerException("only null sims provided");
        }

        /**
         * 
         * @return depending on the sim, flywheel / arm / DC = rad per sec, elevator = height in meters per sec
         */
        public Double getVelocity(){
            if(mDCMotorSim != null) return mDCMotorSim.getAngularVelocityRadPerSec();
            else if(mFlywheelSim != null) return mFlywheelSim.getAngularVelocityRadPerSec();
            else if(mSingleJointedArmSim != null) return mSingleJointedArmSim.getVelocityRadPerSec();
            else if(mElevatorSim != null) return mElevatorSim.getVelocityMetersPerSecond();
            else throw new NullPointerException("only null sims provided");
        }

        /**
         * 
         * @return depending on the sim, flywheel / DC = rad per sec^2, elevator / arm = null
         */
        public Double getAcceleration(){
            if(mDCMotorSim != null) return mDCMotorSim.getAngularAccelerationRadPerSecSq();
            else if(mFlywheelSim != null) return mFlywheelSim.getAngularAccelerationRadPerSecSq();
            else if(mSingleJointedArmSim != null) return null;
            else if(mElevatorSim != null) return null;
            else throw new NullPointerException("only null sims provided");
        }

        /**
         * 
         */
        public void update(double seconds){
            if(mDCMotorSim != null) mDCMotorSim.update(seconds);
            else if(mFlywheelSim != null) mFlywheelSim.update(seconds);
            else if(mSingleJointedArmSim != null) mSingleJointedArmSim.update(seconds);
            else if(mElevatorSim != null) mElevatorSim.update(seconds);
            else throw new NullPointerException("only null sims provided");
        }

        public void setInputVoltage(double volts){
            if(mDCMotorSim != null) mDCMotorSim.setInputVoltage(volts);
            else if(mFlywheelSim != null) mFlywheelSim.setInputVoltage(volts);
            else if(mSingleJointedArmSim != null) mSingleJointedArmSim.setInputVoltage(volts);
            else if(mElevatorSim != null) mElevatorSim.setInputVoltage(volts);
            else throw new NullPointerException("only null sims provided");
        }

    } 

    //#region
    private static final double dt = 0.02;
    
    /**
     * testing
     * @param motor the motor being simulated
     * @param physicalSim the physical component of the simulation, updated with the current position, velocity, and acceleration of the driving motor (not the mechanism it controls)
     * @param gearRatio the gear ratio between the driving motor and the mechanism's final angle / height
     * @param updateState a consumer which takes in a {@link SimSparkConstants}, this is where the code updates the encoder (built in and seperate) and other aspects of the subsystem
     */
    public record SimSparkConstants(
        SparkMax motor,
        PhysicalSimWrapper physicalSim,
        DCMotor gearBox,
        double gearRatio,
        double conversionRate,
        Consumer<SimSparkConstants> updateState
    ) {}

    /**
     * testing
     * @param motor the motor being simulated
     * @param physicalSim the physical component of the simulation, updated with the current position, velocity, and acceleration of the driving motor (not the mechanism it controls)
     * @param gearRatio the gear ratio between the driving motor and the mechanism's final angle / height
     * @param updateState a consumer which takes in a {@link SimTalonConstants}, this is where the code updates the encoder (built in and seperate) and other aspects of the subsystem
     */
    public record SimTalonConstants(
        TalonFX motor,
        PhysicalSimWrapper physicalSim,
        DCMotor gearBox,
        double gearRatio,
        Consumer<SimTalonConstants> updateState
    ) {}

    public interface SimulatedSubsystem {
        default Optional<SimSparkConstants[]> getSparkSimConstants(){
            return Optional.empty();
        }

        default Optional<SimTalonConstants[]> getTalonSimConstants(){
            return Optional.empty();
        }

        /**
         * this function runs once at the end of the simulation step for this subsystem
         */
        default void finalSimUpdate(){}
    }

    //#endregion
    private ArrayList<SimSparkConstants> mSimSparks = new ArrayList<>();
    private ArrayList<SimTalonConstants> mSimTalons = new ArrayList<>();
    private ArrayList<SimulatedSubsystem> mSimSubsystems = new ArrayList<>();

    public MotorSimulationSubsystem(SimulatedSubsystem... mSimulatedSubsystems) {
        super();
        for (SimulatedSubsystem subsystem : mSimulatedSubsystems) {
            subsystem.getSparkSimConstants().ifPresent((motors) -> {
                mSimSparks.addAll(Arrays.asList(motors));
            });

            subsystem.getTalonSimConstants().ifPresent((motors) -> {
                mSimTalons.addAll(Arrays.asList(motors));
            });
            
            mSimSubsystems.add(subsystem);
        }
        
    }

    @Override
    public void simulationPeriodic() {

        for(var mSparkConstants : mSimSparks){

            //TODO gear ratios and conversion
            
            var motor = mSparkConstants.motor;

            var motorSim = new SparkSim(motor, mSparkConstants.gearBox);

            mSparkConstants.physicalSim.setInputVoltage(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
            mSparkConstants.physicalSim.update(dt);
            
            motorSim.iterate(
                mSparkConstants.physicalSim.getVelocity(), 
                RoboRioSim.getVInVoltage(), 
                dt
            );

            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(mSparkConstants.physicalSim.getCurrentDrawAmps()));

            mSparkConstants.updateState.accept(mSparkConstants);
        }

        for(var mTalonConstants : mSimTalons){

            //TODO gear ratios and conversion
            
            // var motor = mSparkConstants.motor;

            var motorSim = mTalonConstants.motor.getSimState();
            
            motorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

            var motorVoltage = motorSim.getMotorVoltage();

            mTalonConstants.physicalSim.setInputVoltage(motorVoltage);
            mTalonConstants.physicalSim.update(dt);

               // apply the new rotor position and velocity to the TalonFX;
            // note that this is rotor position/velocity (before gear ratio), but
            // DCMotorSim returns mechanism position/velocity (after gear ratio)
            motorSim.setRawRotorPosition(
                mTalonConstants.gearRatio * mTalonConstants.physicalSim.getPosition()
            );
            motorSim.setRotorVelocity(
                mTalonConstants.gearRatio * mTalonConstants.physicalSim.getVelocity()
            );

            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(mTalonConstants.physicalSim.getCurrentDrawAmps()));

            mTalonConstants.updateState.accept(mTalonConstants);
        }

        for(var subsystems : mSimSubsystems){
            subsystems.finalSimUpdate();
        }
    }

}
