package com.spartronics4915.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismRenderer extends SubsystemBase {
    

    private Supplier<Distance> elevatorHeightSupplier;
    private Supplier<Angle> armAngleSupplier;
    private Supplier<AngularVelocity> intakeSpeedSupplier;
    private BooleanSupplier intakeTrigger;
    Mechanism2d canvas;
    MechanismLigament2d elev;
    MechanismLigament2d arm;
    MechanismLigament2d intake;
    MechanismLigament2d intakeBB;
    
    public static void generateRenderer(Supplier<Distance> elevatorHeightSupplier, Supplier<Angle> armAngleSupplier, Supplier<AngularVelocity> intakeSpeedSupplier, BooleanSupplier intakeTrigger, String name) {
        new MechanismRenderer(elevatorHeightSupplier, armAngleSupplier, intakeSpeedSupplier, intakeTrigger, name);
    }

    /**
     * 
     * @param elevatorHeightSupplier a supplier which returns the elevator's height from the ground
     * @param armAngleSupplier a supplier which returns the arm angle relative to horizantal (CCW positive) 0 degrees is up
     * @param intakeSpeedSupplier a supplier which returns the intake's angular velocity
     * 
     */
    public MechanismRenderer(Supplier<Distance> elevatorHeightSupplier, Supplier<Angle> armAngleSupplier, Supplier<AngularVelocity> intakeSpeedSupplier, BooleanSupplier intakeTrigger, String name) {
        super();
        this.elevatorHeightSupplier = elevatorHeightSupplier;
        this.armAngleSupplier = armAngleSupplier;
        this.intakeSpeedSupplier = intakeSpeedSupplier;
        this.intakeTrigger = intakeTrigger;

        canvas = new Mechanism2d(3, 3);
        var root = canvas.getRoot("root", 1.5, 0.735800);
        elev = new MechanismLigament2d("elevator", 
            elevatorHeightSupplier.get().in(Meters),
            90,
            5,
            new Color8Bit(Color.kCoral)
        );

        arm = new MechanismLigament2d("arm", 
            0.321500,
            armAngleSupplier.get().in(Degrees),
            5,
            new Color8Bit(Color.kBlue)
        );

        intake = new MechanismLigament2d("intake", 0.25, 90, 5, new Color8Bit(Color.kRed));

        intakeBB = new MechanismLigament2d("beam break", 0.25, -90, 5, new Color8Bit(Color.kBlue));

        arm.append(intake);
        arm.append(intakeBB);
        elev.append(arm);
        root.append(elev);

        SmartDashboard.putData(name, canvas);

    }

    @Override
    public void periodic() {
        elev.setLength(elevatorHeightSupplier.get().in(Meters));
        arm.setAngle(90-(armAngleSupplier.get().in(Degrees)));
        intakeBB.setColor(intakeTrigger.getAsBoolean() ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
        
        double t = MathUtil.inverseInterpolate(
            RPM.of(-500).in(RPM), 
            RPM.of(500).in(RPM), 
            intakeSpeedSupplier.get().in(RPM)
        );
        
        intake.setColor(
            new Color8Bit(Color.fromHSV(
                (int)((1-t)*63.0 + (t)*0.0), 
                255,
                255
            ))
        );
    }
}
