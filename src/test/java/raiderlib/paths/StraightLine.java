package raiderlib.paths;

import java.util.ArrayList;

import raiderlib.path.Path;
import raiderlib.path.WayPoint;

public class StraightLine extends Path {

    @Override
    public ArrayList<WayPoint> get_waypoints() {

        ArrayList<WayPoint> points = new ArrayList<>();
        points.add(new WayPoint(0, 0, 0));
        points.add(new WayPoint(24, 0, 0));
        return points;
    }

}