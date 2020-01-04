package raiderlib;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import org.junit.Test;

import raiderlib.control.DriveCharacterization;
import raiderlib.control.PurePursuitController;
import raiderlib.path.Path;
import raiderlib.path.TrajPoint;
import raiderlib.paths.StraightLine;

public class RaiderLib {

    @Test
    public void testPath() throws IOException {
        FileWriter fout = new FileWriter(new File("output.csv"));
        Path path = new StraightLine();
        PurePursuitController ppc = new PurePursuitController(path, new DriveCharacterization(14, 14, 50), 5);
        fout.append("x,y,velocity,curvature\n");
        for (TrajPoint point : path.get_points())
            fout.append(String.valueOf(point.x) + ',' + String.valueOf(point.y) + ',' + String.valueOf(point.velocity)
                    + ',' + String.valueOf(point.curvature) + '\n');
        fout.close();
    }
}