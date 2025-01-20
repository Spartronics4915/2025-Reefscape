package com.spartronics4915.frc2025.subsystems.Bling;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This one is quite weird. Check out github FinnSpartronics/fbling-sim for the shows and put the exported files into the deploy directory.
 */
public class BlingShow extends BlingSegment {
    private short[][] show;

    public BlingShow(String filename) {
        try {
            this.show = loadFromDeploy(filename);
        } catch(Exception e) {
            e.printStackTrace();
        }
        this.ledLength = show[0].length / 3;
        this.maxLength = show.length - 1;
    }
    
    private short[][] loadFromDeploy(String filename) throws FileNotFoundException, IOException {
        String path = Filesystem.getDeployDirectory().getPath() + "/" + filename;
        System.out.println("Attempting to load " + path);
        BufferedReader reader = new BufferedReader(new FileReader(path));

        String line = reader.readLine();

        ArrayList<ArrayList<Short>> arr = new ArrayList<ArrayList<Short>>();
        
        for (String x : line.split(" ")) {
            ArrayList<Short> tmp = new ArrayList<Short>();
            for (String num : x.split(",")) {
                tmp.add(Short.parseShort(num));
            }
            arr.add(tmp);
        }

        short[][] array = new short[arr.size()][arr.get(0).size()];
        for (int x = 0; x < arr.size(); x++) {
            for (int y = 0; y < arr.get(x).size(); y++) {
                array[x][y] = arr.get(x).get(y);
            }
        }

        reader.close();
        return array;
    }

    public int getR(int index) {
        return show[frame][index * 3];
    }

    public int getG(int index) {
        return show[frame][index * 3 + 1];
    }

    public int getB(int index) {
        return show[frame][index * 3 + 2];
    }

    @Override
    protected void updateLights() {
        for (int i = 0; i < this.ledLength; i++) {
            buffer.setRGB(i, getR(i), getG(i), getB(i));
        }
    }

}
