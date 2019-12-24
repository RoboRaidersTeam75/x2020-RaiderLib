package raiderlib;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.junit.Test;

import raiderlib.path.Path;
import raiderlib.path.TrajPoint;
import raiderlib.paths.CurveLine;

public class RaiderLib {

    @Test
    public void testPath() throws IOException {
        FileWriter fout = new FileWriter(new File("output.csv"));
        Path path = new CurveLine();
        fout.append("x,y,curvature\n");
        for (TrajPoint point : path.get_points())
            fout.append(String.valueOf(point.x) + ',' + String.valueOf(point.y) + ',' + String.valueOf(point.curvature)
                    + '\n');
        fout.close();
    }
}