package simbadproject;

import javax.vecmath.Point3d;
import simbad.sim.Agent;
import simbad.sim.RangeSensorBelt;

/**
 *
 * @author dvrakas
 */
public class Tools {
    
    public static Point3d getSensedPoint(Agent rob,RangeSensorBelt sonars,int sonar){
        double v;
        if (sonars.hasHit(sonar))
            v=rob.getRadius()+sonars.getMeasurement(sonar);
        else
            v=rob.getRadius()+sonars.getMaxRange();
        double x = v*Math.cos(sonars.getSensorAngle(sonar));
        double z = v*Math.sin(sonars.getSensorAngle(sonar));
        return new Point3d(x,0,z);
    }

    public static double wrapToPi(double a){
        if (a>Math.PI)
            return a-Math.PI*2;
        if (a<=-Math.PI)
            return a+Math.PI*2;
        return a;
    } 
}
