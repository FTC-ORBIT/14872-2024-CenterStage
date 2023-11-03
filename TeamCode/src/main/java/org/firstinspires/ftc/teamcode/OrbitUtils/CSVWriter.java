package org.firstinspires.ftc.teamcode.OrbitUtils;

import android.os.Environment;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;

public final class CSVWriter {

    private final Feedback<Float[]> data;
    // private final String fileName;
    private final int lookBackCycles;
    private final String titles;

    private ArrayList<Float> currentLine = new ArrayList<>();

    private boolean hasDataToSave = false;

    public CSVWriter(final int lookBackCycles, final String titles) {
        this.data = new Feedback<Float[]>(new Float[] {}, lookBackCycles);
        this.lookBackCycles = lookBackCycles;
        this.titles = titles;
    }

    public void addData(final Float[] line) {
        this.data.update(line);

        hasDataToSave = true;
    }

    public void addDataToLine(final float... data) {
        for (final float x : data)
            currentLine.add(x);
        hasDataToSave = true;
    }

    public void endLine() {
        final Float[] currentLine = this.currentLine.toArray(new Float[this.currentLine.size()]);
        this.data.update(currentLine);
        this.currentLine.clear();
    }

    public void saveFile() {
        if (!hasDataToSave)
            return;

        try {
            final File file = new File(String.format("%s/FIRST/robot-data.csv",
                         Environment.getExternalStorageDirectory().getAbsolutePath()));
            final PrintWriter writer = new PrintWriter(file);

            // Write titles
            writer.print(this.titles);
            writer.print("\n");

            // Write values
            for (int i = lookBackCycles; i >= 0; i--) {
                if (this.data.getFeedback(i).length == 0) {
                    continue;
                }

                for (final float value : this.data.getFeedback(i)) {
                    writer.print(value + "\t");
                }
                writer.print("\n");
            }

            writer.close();

        } catch (final FileNotFoundException e) {
            System.out.println("CSV file not found. :(");
        }

        hasDataToSave = false;
    }
}