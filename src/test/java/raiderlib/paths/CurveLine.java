package raiderlib.paths;

import java.util.ArrayList;

import raiderlib.path.Path;
import raiderlib.path.WayPoint;

public class CurveLine extends Path {

    @Override
    public ArrayList<WayPoint> get_waypoints() {
        ArrayList<WayPoint> points = new ArrayList<>();
        points.add(new WayPoint(0, 0, Math.toRadians(0)));
        points.add(new WayPoint(12, 0, Math.toRadians(90)));
        points.add(new WayPoint(24, 24, Math.toRadians(90)));
        return points;
    }

}