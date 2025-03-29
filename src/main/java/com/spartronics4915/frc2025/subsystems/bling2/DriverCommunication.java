package com.spartronics4915.frc2025.subsystems.bling2;

import com.spartronics4915.frc2025.Constants;
import com.spartronics4915.frc2025.Constants.ArmConstants;
import static com.spartronics4915.frc2025.Constants.BlingConstants.BAD;
import static com.spartronics4915.frc2025.Constants.BlingConstants.FRAME_WAIT;
import static com.spartronics4915.frc2025.Constants.BlingConstants.MATCH_END;
import static com.spartronics4915.frc2025.Constants.BlingConstants.OFF;
import static com.spartronics4915.frc2025.Constants.BlingConstants.ORANGE;
import static com.spartronics4915.frc2025.Constants.BlingConstants.PURPLE;
import static com.spartronics4915.frc2025.Constants.BlingConstants.SHOW_SPARTRONICS;
import com.spartronics4915.frc2025.Constants.ElevatorConstants;
import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverCommunication extends BlingSegment {
    private Constants constants;
    private SwerveSubsystem swerve;
    private LimelightVisionSubsystem vision;
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private DynamicsCommandFactory dynamics;

    private int alertFrames = 0;

    private BlingSegment current = OFF;


   /*  public static enum Region {
        REEF(
            new Translation2d[] {new Translation2d(5, 4), new Translation2d(4, 4), new Translation2d(4.5, 4.5), new Translation2d(4.5, 3.5)}, 
            new Translation2d[] {new Translation2d(13.5, 4), new Translation2d(12.5, 4), new Translation2d(13, 4.5), new Translation2d(13, 4.5)}
        ),
        PROCESSOR(
            new Translation2d[] {new Translation2d(6, .5)}, 
            new Translation2d[] {new Translation2d(11.5, 7.5)}
        ),
        BARGE(
            new Translation2d[] {
                new Translation2d(8.8, 7.2),
                new Translation2d(8.8, 6.1),
                new Translation2d(8.8, 5)
            }, 
            new Translation2d[] {
                new Translation2d(8.8, 3),
                new Translation2d(8.8, 1.9),
                new Translation2d(8.8, .8)
            }
        ),
        CORAL_STATION(
            new Translation2d[] {new Translation2d(.5, 7.5), new Translation2d(.5, .5)}, 
            new Translation2d[] {new Translation2d(17, 7.5), new Translation2d(17, .5)}
        );

        public final Translation2d[] redPositions;        
        public final Translation2d[] bluePositions;

        private Region(Translation2d[] blue, Translation2d[] red) {
            this.bluePositions = blue;
            this.redPositions = red;
        }
    } */

  

    /**
     * @param length Length of the segment
     * @param subsystems In no particular order, SwerveSubsystem, LimelightVisionSubsystem, ArmSubsystem, ElevatorSubsystem, DynamicsCommandFactory
     */
    public DriverCommunication(int length, Object... subsystems) {
        this.ledLength = length;
        for (Object subsystem : subsystems) {
            if (subsystem instanceof SwerveSubsystem) this.swerve = (SwerveSubsystem) subsystem;
            if (subsystem instanceof LimelightVisionSubsystem) this.vision = (LimelightVisionSubsystem) subsystem;
            if (subsystem instanceof ArmSubsystem) this.arm = (ArmSubsystem) subsystem;
            if (subsystem instanceof ElevatorSubsystem) this.elevator = (ElevatorSubsystem) subsystem;
            if (subsystem instanceof DynamicsCommandFactory) this.dynamics = (DynamicsCommandFactory) subsystem;
        }
    }
     /* 
    public static Region getClosestRegion(SwerveSubsystem swerve) {
        Region closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Region reg : Region.values()) {
            for (Translation2d position : (DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get().equals(Alliance.Blue) ? reg.bluePositions : reg.redPositions) : reg.redPositions)) {
                if (swerve.getPose().getTranslation().getDistance(position) < closestDistance) {
                    closest = reg;
                    closestDistance = swerve.getPose().getTranslation().getDistance(position);
                }
            }
        }
        return closest;
    } */
        

    @Override
    protected void updateLights() {
        if ((!Robot.AUTO_TIMER.hasElapsed(0.01) && !Robot.TELEOP_TIMER.hasElapsed(0.01)) && vision != null) { // Match has started
            current = vision.isInitialPoseSet() ? SHOW_SPARTRONICS : PURPLE; 
        } else if (DriverStation.isAutonomous()) {


            if (dynamics.funnelDetect() == false) {
                current = PURPLE;
            }

        }else if (ArmConstants.kMinAngle.getDegrees() > arm.getPosition().getDegrees() == true ||
         ArmConstants.kMaxAngle.getDegrees() < arm.getPosition().getDegrees() == true){
            current = ORANGE;

        }else if (ElevatorConstants.minHeight > elevator.getPosition() == true || ElevatorConstants.maxHeight < elevator.getPosition()){
            current = ORANGE;

            //arm and elevator accuracy?
        // } else if (vision != null && vision.newMegaTag1Reading()) {
        //     current = CYAN;
        //     alertFrames = 10;
        //} else if (alertFrames > 0 && alertFrames % 2 == 0) {
            //current = CYAN;
        } else if (Robot.TELEOP_TIMER.hasElapsed(140)) { // Match has ended, play show
            current = MATCH_END;

        } else if (Robot.TELEOP_TIMER.hasElapsed(135)) { // Match has ended, show match end alert.
            current = BAD;
        }
        else {
            /*Region closest = getClosestRegion(this.swerve);
            double elevHeight = elevator.getPosition();
            Rotation2d armRotation = arm.getPosition();
              switch (closest) {
                case PROCESSOR: // Extension of Reef zone
                case REEF:
                    Pose2d closestAprilTag = AlignToReef.getClosestReefAprilTag(swerve.getPose());
                    int index = AlignToReef.allReefTagPoses.indexOf(closestAprilTag);
                    switch(index) {
                        case 1:
                        case 7:
                            current = WHITE;
                            break;
                        case 0:
                        case 2:
                        case 8:
                        case 6:
                            current = ORANGE;
                            break;
                        case 3:
                        case 5:
                        case 9:
                        case 11:
                            current = GREEN;
                            break;
                        case 4:
                        case 10:
                            current = PURPLE;
                            break;
                        default:
                            current = OFF;
                            break;
                    } 

                    break;
                case CORAL_STATION:
                    boolean subsystemsInCorrectSpot = Math.abs(armRotation.minus(LOAD.getArmAngle()).getDegrees()) < ARM_THRESHOLD // If arm in correct spot
                                    && Math.abs(elevHeight - LOAD.getElevatorHeight()) < ELEVATOR_THRESHOLD; // And elevator in correct spot
                    boolean robotInRightSpot = true; // TODO: Figure out robot in right spot

                    if (dynamics.funnelDetect()) {
                        current = RAINBOW;
                    } else {
                        if (subsystemsInCorrectSpot && robotInRightSpot) current = GOOD;
                        else if (subsystemsInCorrectSpot) current = PURPLE; // Robot needs to move
                        else if (robotInRightSpot) current = WARN; // Mechanisms need to move
                        else current = OFF; // Should never be that
                    } 

                    break;
                case BARGE:
                     Climber Lights
                    if (DriverStation.getLocation().isEmpty()) {
                        current = OFF;
                        break;
                    }
                    int location = DriverStation.getLocation().getAsInt() - 1;
                    double distance;
                    boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);

                    if (isBlue) distance = OrientTowardsNearestPOIConstants.BARGE_BLUE_CAGE_POSITIONS[location].minus(swerve.getPose().getTranslation()).getMeasureY().in(Meters);
                    else  distance = OrientTowardsNearestPOIConstants.BARGE_RED_CAGE_POSITIONS[location].minus(swerve.getPose().getTranslation()).getMeasureY().in(Meters);
                
                    // If showing red, too far right, go further left.
                    // If showing blue, too far left, go further right.

                    if (distance <= -BARGE_ALIGNMMENT_THRESHOLD) current = isBlue ? BLUE : RED;
                    else if (distance >= BARGE_ALIGNMMENT_THRESHOLD) current = isBlue ? RED : BLUE;
                    else current = RAINBOW; // It's climbin' time.
                     

                    current = SHOW_RAINBOW_FUN;

                    break;
                
                default:
                    current = OFF;
                }
            */}

    

        current.incrementFrame(FRAME_WAIT);
        current.buffer = this.buffer;
        current.updateLights();

        alertFrames--;
    }

    /*public static Command setAutoSegmentCommand(BlingSegment segment) {
        return Commands.runOnce(() -> autoSegment = segment);
    }*/

}
