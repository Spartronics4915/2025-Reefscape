package com.spartronics4915.frc2025.subsystems.bling2;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This one is quite weird. Check out github FinnSpartronics/fbling-sim for the shows and put the exported files into the deploy directory.
 */
public class BlingShow extends BlingSegment {
    private byte[][] show;

    public BlingShow(String filename) {
        try {
            this.show = loadFromDeploy(filename);
        } catch(Exception e) {
            e.printStackTrace();
        }
        this.ledLength = show[0].length / 3;
        this.maxLength = show.length - 1;
    }
    
    private byte[][] loadFromDeploy(String filename) throws IOException, OutOfMemoryError, SecurityException {
        System.out.println("Attempting to load " + Filesystem.getDeployDirectory().getPath() + "/" + filename);

        byte[] arr = Files.readAllBytes(Path.of(Filesystem.getDeployDirectory().getPath() + "/" + filename));

        ArrayList<ArrayList<Byte>> arrList = new ArrayList<ArrayList<Byte>>();
        
        ArrayList<Byte> current = new ArrayList<Byte>();
        for (byte b : arr) {
            if (b == 0) {
                arrList.add(current);
                current = new ArrayList<Byte>();
            } else current.add(Byte.valueOf(b));
        }

        byte[][] output = new byte[arrList.size()][arrList.get(0).size()];
        for (int x = 0; x < arrList.size(); x++) {
            for (int y = 0; y < arrList.get(0).size(); y++) {
                try {
                    output[x][y] = arrList.get(x).get(y);
                } catch (Exception e) {
                    break;
                }
            }
        }

        return output;
    }

    public int getR(int index) {
        return show[frame][index * 3] & 0xff;
    }

    public int getG(int index) {
        return show[frame][index * 3 + 1] & 0xff;
    }

    public int getB(int index) {
        return show[frame][index * 3 + 2] & 0xff;
    }

    @Override
    protected void updateLights() {
        for (int i = 0; i < this.ledLength; i++) {
            buffer.setRGB(i, getR(i), getG(i), getB(i));
        }
    }

}
