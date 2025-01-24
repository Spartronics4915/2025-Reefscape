package com.spartronics4915.frc2025.util;

import static edu.wpi.first.units.Units.Seconds;

import java.io.Closeable;
import java.io.IOException;
import java.io.InvalidObjectException;
import java.util.ArrayList;
import java.util.DuplicateFormatFlagsException;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.spartronics4915.frc2025.Constants.OI;

public final class RumbleFeedbackHandler{
    public enum FeedbackControllers{
        DRIVER,
        OPERATOR,
        DEBUG;

        public RumbleController internal;

        private FeedbackControllers() {}
    }

    public record RumbleFeedback(RumbleType type, double strength) {}

    private static final ArrayList<RumbleController> rumbleControllers = new ArrayList<>();

    
    /**
     * these should be created and accessed statically
     */
    public static class RumbleController{
        private final XboxController mController;
        private RumbleFeedback mCurrentFeedback;
        private boolean isActive = false;
        private boolean change = false;


        public RumbleController(XboxController controller) {
            super();
            mController = controller;
            rumbleControllers.add(this);
        }
        public RumbleController(CommandXboxController controller) {this(controller.getHID());}

        /**
         * be absolutely certain that, when this is used you run disable(). otherwise the controllers will vibrate continously
         */
        public void setFeedback(RumbleFeedback feedback) {
            mCurrentFeedback = feedback;
            isActive = true;
            change = true;
        }

        public void disable(){
            isActive = false;
            change = true;
        }

        public void periodicBehavior(){
            //check if change in strength or anything
            if (change) {
                //this means that the controller should stop rumbling (allows it to take in a setting to one of the rumbles)
                mController.setRumble(RumbleType.kBothRumble, 0.0);
                System.out.println(mController.getPort() + " : set " + mCurrentFeedback.type + " to 0.0");

                //this means that the controller should start vibrating
                if (isActive) {
                    System.out.println(mController.getPort() + " : set " + mCurrentFeedback.type + " to " + mCurrentFeedback.strength);
                    mController.setRumble(mCurrentFeedback.type, mCurrentFeedback.strength);
                }
            }
            change = false;
            
        }
        
        public boolean equals(RumbleController obj) {
            if (rumbleControllers == null) {
                return false;
            }
            return obj.mController.getPort() == this.mController.getPort();
        }
    }


    public static Command getRumbleCommand(RumbleController controller, RumbleFeedback feedback){
        return Commands.startEnd(() ->  controller.setFeedback(feedback), controller::disable);
    }

    public static Command getRumbleCommand(RumbleController controller, RumbleFeedback feedback, double seconds){
        return getRumbleCommand(controller, feedback).withTimeout(seconds);
    }

    public static Command getRumbleCommand(RumbleController controller, RumbleFeedback feedback, Time timeout){
        return getRumbleCommand(controller, feedback, timeout.in(Seconds));
    }

    public static void handleControllers(){
        rumbleControllers.forEach((c) -> c.periodicBehavior());
    }

    public static void onDisable() {
        rumbleControllers.forEach((c) -> c.disable());
    }
}
