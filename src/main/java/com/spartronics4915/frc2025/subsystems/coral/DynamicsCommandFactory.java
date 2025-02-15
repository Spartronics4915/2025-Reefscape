package com.spartronics4915.frc2025.subsystems.coral;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DynamicsCommandFactory {

    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public enum RobotPosition {
        INTAKE_POSITION(Rotation2d.fromDegrees(0), (0)),
        L4_SCORE(Rotation2d.fromDegrees(0), (0)),
        STOW(Rotation2d.fromDegrees(0), (0));

        public Rotation2d armAngle;
        public double metersOfElevation;
        //add more or less positions later

        private RobotPosition(Rotation2d armAngle, double metersOfElevation) {
            this.armAngle = armAngle;
            this.metersOfElevation = metersOfElevation;
        }
        
    }

    public Command moveToStateCommand(RobotPosition newState){
       return Commands.parallel(armSubsystem.setMechanismAngleCommand(newState.armAngle),
                          elevatorSubsystem.setSetPointCommand(newState.metersOfElevation));
    }

}

    
   







    

