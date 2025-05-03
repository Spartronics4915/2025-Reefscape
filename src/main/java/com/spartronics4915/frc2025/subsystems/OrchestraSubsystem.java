package com.spartronics4915.frc2025.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
    private Orchestra orchestra;
    private String track = "";

    /**
     * Each device will fill a track, starting at 0
     */
    public OrchestraSubsystem(ParentDevice... devices) {
        orchestra = new Orchestra();
        for (ParentDevice device : devices) orchestra.addInstrument(device);

        logSfxCommands();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Orchestra/Is Playing?", orchestra.isPlaying());
        SmartDashboard.putNumber("Orchestra/Time", orchestra.getCurrentTime());
        SmartDashboard.putString("Orchestra/Track", track);
    }

    /**
     * Loads a track relative to the deploy/orchestra directory
     */
    public void loadTrack(String path) {
        track = path;

        if (!path.endsWith(".chrp")) path += ".chrp";
        path = Filesystem.getDeployDirectory() + "/orchestra/" + path;
        var status = orchestra.loadMusic(path);

        if (!status.isOK()) {
            System.out.println("Problem loading track! " + status.getDescription());
        }
    }

    public void play() {
        var status = orchestra.play();

        if (!status.isOK()) {
            System.out.println("Problem playing! " + status.getDescription());
        }
    }

    public void pause() {
        var status = orchestra.pause();

        if (!status.isOK()) {
            System.out.println("Problem pausing! " + status.getDescription());
        }
    }

    public void stop() {
        var status = orchestra.stop();

        if (!status.isOK()) {
            System.out.println("Problem stopping! " + status.getDescription());
        }
    }

    /**
     * Load a track and play it instantly
     */
    public void quickPlay(String path) {
        if (orchestra.isPlaying()) stop();
        loadTrack(path);
        play();
    }

    /**
     * Generates a command that loads and plays a sound effect. The command ends once the sound effect has finished playing.
     */
    public Command playSoundEffectCommand(SFX sfx) {
        return Commands.sequence(
            Commands.runOnce(() -> quickPlay(sfx.path), this),
            Commands.waitSeconds(sfx.duration),
            Commands.runOnce(this::stop, this)
        )
        .handleInterrupt(this::stop)
        .ignoringDisable(true)
        .withName("Orchestra: " + sfx.path);
    }

    public enum SFX {
        MATCH_START("match-start", 2.2),
        FREDDY("freddy", 9.0),
        ;

        String path;
        double duration;

        private SFX(String path, double duration) {
            this.path = path;
            this.duration = duration;
        }
    }

    private void logSfxCommands() {
        for (SFX sfx : SFX.values()) SmartDashboard.putData("Orchestra/" + sfx.path, playSoundEffectCommand(sfx));
    }
}
