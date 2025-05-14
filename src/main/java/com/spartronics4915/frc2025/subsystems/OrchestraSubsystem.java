package com.spartronics4915.frc2025.subsystems;

import java.io.File;
import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;
import java.util.stream.Collectors;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {
    private Orchestra orchestra;
    private String track = "";
    private Song song;
    private ArrayList<Song> playlist = Playlist.ALL.songs;
    private SendableChooser<Playlist> playlistSelector;
    private boolean shuffled;

    /**
     * Each device will fill a track, starting at 0
     */
    public OrchestraSubsystem(ParentDevice... devices) {
        orchestra = new Orchestra();
        for (ParentDevice device : devices) orchestra.addInstrument(device);

        logSfxCommands();
        createPlaylistSelector();
        initSongPlayer();
        logSongDatabaseSize();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Orchestra/Is Playing?", orchestra.isPlaying());
        SmartDashboard.putNumber("Orchestra/Time", orchestra.getCurrentTime());
        SmartDashboard.putString("Orchestra/Track", track);

        if (track == song.path) {
            SmartDashboard.putString("Orchestra/Song/Timestamp", formatTime(orchestra.getCurrentTime()));
            SmartDashboard.putNumber("Orchestra/Song/Progress", orchestra.getCurrentTime() / song.duration);
            if (orchestra.getCurrentTime() > song.duration) next();
        }

        SmartDashboard.putBoolean("Orchestra/Song/Playing", (track == song.path) && (orchestra.isPlaying()));
    }

    private void initSongPlayer() {
        SmartDashboard.putString("Orchestra/Song/Cover", "#b2b2b2");
        SmartDashboard.putString("Orchestra/Song/Now Playing", "None");
        SmartDashboard.putString("Orchestra/Song/by", "");
        SmartDashboard.putString("Orchestra/Song/found in", "");
        SmartDashboard.putString("Orchestra/Song/Track", "0/0");
        SmartDashboard.putString("Orchestra/Song/Timestamp", "0:00");
        SmartDashboard.putNumber("Orchestra/Song/Progress", 0);
        SmartDashboard.putString("Orchestra/Song/Duration", "0:00");
        SmartDashboard.putBoolean("Orchestra/Song/Playing", false);
        SmartDashboard.putData("Orchestra/Song/    \u23EF    ", Commands.defer(() -> { //18 spaces
            return Commands.runOnce(() -> {
                if (track != song.path) loadTrack(song.path);
                if (orchestra.isPlaying()) pause();
                else play();
            });
        }, Set.of()).ignoringDisable(true));
        SmartDashboard.putData("Orchestra/Song/    \u23ED    ", Commands.runOnce(this::next).ignoringDisable(true));
        SmartDashboard.putData("Orchestra/Song/    \u23EE    ", Commands.runOnce(this::previous).ignoringDisable(true));
        SmartDashboard.putData("Orchestra/Song/Shuffle", Commands.startEnd(
            () -> {
                Collections.shuffle(playlist);
                updateSongPlayer(song);
                updateSongList(playlist);
                shuffled = true;
            },
            () -> {
                Collections.sort(playlist);
                updateSongPlayer(song);
                updateSongList(playlist);
                shuffled = false;
            }
        ).ignoringDisable(true));

        loadSong(playlist.get(0));
    }

    private void loadSong(Song song) {
        if (orchestra.isPlaying()) stop();
        loadTrack(song.path);
        updateSongPlayer(song);
        this.song = song;
    }

    private void updateSongPlayer(Song song) {
        int color = song.path.hashCode() % 0x1000000;
        String hexCode = Integer.toHexString(color);
        while (hexCode.length() < 6) hexCode = "0" + hexCode;
        SmartDashboard.putString("Orchestra/Song/Cover", "#" + hexCode);
        SmartDashboard.putString("Orchestra/Song/Now Playing", song.title);
        SmartDashboard.putString("Orchestra/Song/by", song.artist);
        String index = "" + (playlist.indexOf(song) + 1);
        while (index.length() < ("" + playlist.size()).length()) index = "0" + index;
        SmartDashboard.putString("Orchestra/Song/Track", index + "/" + playlist.size());
        SmartDashboard.putString("Orchestra/Song/Duration", formatTime(song.duration));

        ArrayList<String> foundIn = new ArrayList<>();
        for (Playlist p : Playlist.values()) if (p.songs.indexOf(song) > -1 && !p.equals(Playlist.ALL)) foundIn.add(p.name);
        SmartDashboard.putString("Orchestra/Song/found in", String.join(", ", foundIn.toArray(new String[foundIn.size()])));

        logSongSize(song);

    }

    private String formatTime(double time) {
        int rounded = (int) Math.floor(time);
        int seconds = rounded % 60;
        int minutes = rounded / 60;
        String secondsString = "" + seconds;
        if (secondsString.length() == 1) secondsString = "0" + secondsString;
        return minutes + ":" + secondsString;
    }

    /**
     * Loads a track relative to the deploy/orchestra directory
     */
    private void loadTrack(String path) {
        track = path;

        if (!path.endsWith(".chrp")) path += ".chrp";
        path = Filesystem.getDeployDirectory() + "/orchestra/" + path;
        var status = orchestra.loadMusic(path);

        if (!status.isOK()) {
            System.out.println("Problem loading track! " + status.getDescription());
        }
    }

    public void next() {
        if (song == null) return;
        int index = playlist.indexOf(song);
        if (index < 0) return;
        if (index + 1 == playlist.size()) index = 0;
        else index++;
        boolean isPlaying = orchestra.isPlaying();
        loadSong(playlist.get(index));
        updateSongList(playlist);
        if (isPlaying) play();
    }

    public void previous() {
        if (song == null) return;
        int index = playlist.indexOf(song);
        if (index < 0) return;
        if (index == 0) index = playlist.size() - 1;
        else index--;
        boolean isPlaying = orchestra.isPlaying();
        loadSong(playlist.get(index));
        updateSongList(playlist);
        if (isPlaying) play();
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

        public String path;
        public double duration;

        private SFX(String path, double duration) {
            this.path = path;
            this.duration = duration;
        }
    }

    private void logSfxCommands() {
        for (SFX sfx : SFX.values()) SmartDashboard.putData("Orchestra/" + sfx.path, playSoundEffectCommand(sfx));
    }

    public enum Song {
        RAINBOW_TYLENOL("rainbow", "Rainbow Tylenol", "Kitsune^2", 107),
        AI_BOMB("aibomb", "Artificial Intelligence Bomb", "naruto", 158),
        FNAF("freddy", "FNAF Music Box", "Scott Cawthon", 18),
        NIGHT("night", "Night of Nights", "beatMARIO", 190),
        BAD_APPLE("badapple", "Bad Apple!!", "ZUN", 169),
        MEGALOVANIA("megalovania", "Megalovania", "Toby Fox", 274),
        SINK("sink", "sink to the deep sea world", "Chroma", 238),
        HOT_MILK("hotmilk", "Hot Milk", "Snail's House", 232),
        LAGTRAIN("lagtrain", "Lagtrain", "inabakumori", 248),
        FANTASY("fantasy", "Chaoz Fantasy", "ParagonX9", 60),
        ABANDON("abandon", "Abandon Ship", "Simon Chylinski", 27.5),
        UNDYNE("undyne", "Battle Against a True Hero", "Toby Fox", 93),
        WEEZER("weezer", "Buddy Holly", "Weezer", 158),
        SUGARPLUM("sugarplum", "Dance of the Sugar Plum Fairy", "Pyotr Ilyich Tchaikovsky", 128), //fixme
        BALATRO("balatro", "Balatro OST", "Luis Clemente", 248),
        FISHES("weird-fishes", "Weird Fishes / Arpeggi", "Radiohead", 57.7),
        STRAWBERRY("strawberry", "Strawberry Crisis!!", "ZUN", 342),
        FURIES("whereabouts", "Pure Furies ~ Whereabouts of the Heart", "ZUN", 395),
        EASTERN("eastern", "Eastern Judgement in the Sixtieth Year", "ZUN", 258),
        MARS("cats-on-mars", "Cats on Mars", "Seatbelts", 29.2),
        HORSE("horse", "Riding - Night", "Manaka Kataoka, Yasuaki Iwata, Hajime Wakai", 64),
        LUNATIC("lunatic-eyes", "Lunatic Eyes ~ Invisible Full Moon", "ZUN", 85.2),
        HOT_TO_GO("hottogo", "Hot To Go", "Chappell Roan", 4.5),
        QUEEN("queen", "Attack of the Killer Queen", "Toby Fox", 242.5),
        ARIA_MATH("ariamath", "Aria Math", "C418", 317.6),
        FAREWELL("farewell", "Beyond the Heart", "Lena Raine", 25.8),
        // DIALUP("dialup", "Dialup", "Your Modem", 18),
        STEREO_MADNESS("stereo-madness", "Stereo Madness", "DJ Nate", 85.5),
        BUBBLEGUM("bubblegum", "Bubblegum KK", "KK Slider", 135),
        CAUTION("caution", "No Time for Caution", "Hans Zimmer", 182),
        FIRE("fire", "Through the Fire and Flames", "Dragonforce", 432.5),
        ;

        public String path, title, artist;
        public double duration;

        private Song(String path, String title, String artist, double duration) {
            this.path = path;
            this.title = title;
            this.artist = artist;
            this.duration = duration;
        }
    }

    public enum Playlist {
        ALL("All", Song.values()),
        MIX_1("Mix #01", Song.RAINBOW_TYLENOL, Song.AI_BOMB, Song.FNAF, Song.NIGHT, Song.BAD_APPLE),
        MIX_2("Mix #02", Song.MEGALOVANIA, Song.SINK, Song.HOT_MILK, Song.LAGTRAIN, Song.FANTASY),
        MIX_3("Mix #03", Song.ABANDON, Song.UNDYNE, Song.WEEZER, Song.SUGARPLUM, Song.BALATRO),
        MIX_4("Mix #04", Song.FISHES, Song.STRAWBERRY, Song.FURIES, Song.EASTERN, Song.MARS),
        MIX_5("Mix #05", Song.HORSE, Song.LUNATIC, Song.HOT_TO_GO, Song.QUEEN, Song.ARIA_MATH),
        MIX_6("Mix #06", Song.FAREWELL, Song.STEREO_MADNESS, Song.BUBBLEGUM, Song.CAUTION, Song.FIRE),
        VIDEO_GAME("Video Game Songs", Song.FNAF, Song.NIGHT, Song.BAD_APPLE, Song.MEGALOVANIA, Song.FANTASY,
            Song.ABANDON, Song.UNDYNE, Song.BALATRO, Song.STRAWBERRY, Song.FURIES,
            Song.EASTERN, Song.HORSE, Song.LUNATIC, Song.QUEEN, Song.ARIA_MATH,
            Song.FAREWELL, Song.STEREO_MADNESS, Song.BUBBLEGUM),
        ENERGETIC("Energetic", Song.RAINBOW_TYLENOL, Song.AI_BOMB, Song.NIGHT, Song.BAD_APPLE, Song.SINK,
            Song.FANTASY, Song.UNDYNE, Song.STRAWBERRY, Song.QUEEN, Song.FIRE),
        INTERNET("From the Internet", Song.RAINBOW_TYLENOL, Song.FNAF, Song.BAD_APPLE, Song.MEGALOVANIA, Song.HOT_MILK,
            Song.WEEZER, Song.MARS, Song.FIRE),
        TOUHOU("Touhou Jams", Song.NIGHT, Song.BAD_APPLE, Song.STRAWBERRY, Song.FURIES, Song.EASTERN,
            Song.LUNATIC),
        FAVS("Evan's Picks", Song.LAGTRAIN, Song.HORSE, Song.FAREWELL, Song.SINK, Song.AI_BOMB),
        SHORT("Jingles", Song.FNAF, Song.ABANDON, Song.MARS, Song.HOT_TO_GO, Song.FAREWELL),
        LONG("Long Songs", Song.MEGALOVANIA, Song.SINK, Song.HOT_MILK, Song.BALATRO, Song.STRAWBERRY,
            Song.FURIES, Song.EASTERN, Song.QUEEN, Song.ARIA_MATH, Song.FIRE),
        NORMAL("\"Normal\" Music", Song.WEEZER, Song.SUGARPLUM, Song.FISHES, Song.HOT_TO_GO),
        GOOD("Works Well", Song.LAGTRAIN, Song.MEGALOVANIA, Song.FNAF, Song.MARS, Song.HORSE,
            Song.BUBBLEGUM),
        MANY_TRACKS("More Motors Needed", Song.RAINBOW_TYLENOL, Song.AI_BOMB, Song.NIGHT, Song.BAD_APPLE, Song.SINK,
            Song.HOT_MILK, Song.UNDYNE, Song.SUGARPLUM, Song.BALATRO, Song.STRAWBERRY,
            Song.FURIES, Song.EASTERN, Song.LUNATIC, Song.QUEEN, Song.FAREWELL,
            Song.CAUTION, Song.FIRE)
        ;

        public String name;
        public ArrayList<Song> songs;

        private Playlist(String name, Song... songs) {
            this.name = name;
            this.songs = new ArrayList<>();
            Collections.addAll(this.songs, songs);
        }
    }

    private void createPlaylistSelector() {
        playlistSelector = new SendableChooser<>();
        for (Playlist playlist : Playlist.values()) {
            playlistSelector.addOption(playlist.name, playlist);
        }
        playlistSelector.setDefaultOption("All", Playlist.ALL);
        playlistSelector.onChange((playlist) -> {
            this.playlist = new ArrayList<>(playlist.songs);
            if (orchestra.isPlaying() && track == song.path) stop();
            song = playlist.songs.get(0);
            if (shuffled) Collections.shuffle(this.playlist);
            updateSongPlayer(song);
            updateSongList(this.playlist);
        });
        updateSongList(Playlist.ALL.songs);
        SmartDashboard.putData("Orchestra/Song/Playlist", playlistSelector);
    }

    private void updateSongList(ArrayList<Song> songs) {
        String trackList = "";
        for (Song song : songs) {
            trackList += (song.equals(this.song) ? "\u25C9 " : "\u25CB ") + song.title + " (" + formatTime(song.duration) + ")\n";
        }
        trackList = trackList.substring(0, trackList.length() - 1);
        SmartDashboard.putString("Orchestra/Song/Track List", trackList);
    }

    private void logSongDatabaseSize() {
        File database = new File(Filesystem.getDeployDirectory() + "/orchestra");
        File[] files = database.listFiles();
        long totalBytes = 0;
        for (File file : files) {
            totalBytes += file.length();
        }
        SmartDashboard.putString("Orchestra/Song/Database Size", humanReadableByteCountBin(totalBytes));
    }

    private void logSongSize(Song song) {
        File songFile = new File(Filesystem.getDeployDirectory() + "/orchestra/" + song.path + ".chrp");
        SmartDashboard.putString("Orchestra/Song/Song Size", humanReadableByteCountBin(songFile.length()));
    }

    // https://stackoverflow.com/questions/3758606/how-can-i-convert-byte-size-into-a-human-readable-format-in-java/3758880#3758880
    public static String humanReadableByteCountBin(long bytes) {
    long absB = bytes == Long.MIN_VALUE ? Long.MAX_VALUE : Math.abs(bytes);
    if (absB < 1024) {
        return bytes + " B";
    }
    long value = absB;
    CharacterIterator ci = new StringCharacterIterator("KMGTPE");
    for (int i = 40; i >= 0 && absB > 0xfffccccccccccccL >> i; i -= 10) {
        value >>= 10;
        ci.next();
    }
    value *= Long.signum(bytes);
    return String.format("%.1f %cB", value / 1024.0, ci.current());
}
}
