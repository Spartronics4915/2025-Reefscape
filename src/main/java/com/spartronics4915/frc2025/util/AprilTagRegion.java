package com.spartronics4915.frc2025.util;

import java.util.Arrays;

public class AprilTagRegion {
    private enum Regions {
        STATION(new int[]{1, 2}, new int[]{12, 13}),
        PROCESSOR(new int[]{3}, new int[]{16}),
        BARGE(new int[]{4, 5}, new int[]{14, 15}),
        REEF(new int[]{6, 7, 8, 9, 10, 11}, new int[]{17, 18, 19, 20, 21, 22}),
        EMPTY(new int[]{}, new int[]{});

        public final int[] red, blue;

        private Regions(int[] red, int[] blue) {
            this.red = red;
            this.blue = blue;
        }
    }

    private int[] red, blue;

    public static final AprilTagRegion kStation = new AprilTagRegion(Regions.STATION);
    public static final AprilTagRegion kProcessor = new AprilTagRegion(Regions.PROCESSOR);
    public static final AprilTagRegion kBarge = new AprilTagRegion(Regions.BARGE);
    public static final AprilTagRegion kReef = new AprilTagRegion(Regions.REEF);
    public static final AprilTagRegion kEmpty = new AprilTagRegion(Regions.EMPTY);

    private AprilTagRegion(Regions region) {
        this.red = region.red;
        this.blue = region.blue;
    }

    private AprilTagRegion(int[] red, int[] blue) {
        this.red = red;
        this.blue = blue;
    }

    public int[] red() {
        return red;
    }

    public int[] blue() {
        return blue;
    }

    public int[] both() {
        int[] both = new int[red.length + blue.length];
        System.arraycopy(red, 0, both, 0, red.length);
        System.arraycopy(blue, 0, both, red.length, blue.length);
        return both;
    }

    public AprilTagRegion and(AprilTagRegion other) {
        int[] newRed = Arrays.copyOf(red, red.length + other.red.length);
        System.arraycopy(other.red, 0, newRed, red.length, other.red.length);
        int[] newBlue = Arrays.copyOf(blue, blue.length + other.blue.length);
        System.arraycopy(other.blue, 0, newBlue, blue.length, other.blue.length);
        return new AprilTagRegion(newRed, newBlue);
    }
}