package simbadproject;

import java.util.ArrayList;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import simbad.sim.Agent;
import simbad.sim.LightSensor;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

public class MyRobot extends Agent {
    public enum robotState {
        Forward, Orientation, Follow, Stop
    }  
    private robotState state; 
    private final ArrayList<Float> previous_center_observations;
    private final RangeSensorBelt sonars;  
    private final LightSensor light_center;
    private final LightSensor light_left;
    private final LightSensor light_right;
    private float previous_left;
    private float previous_right;
    
    private static final double SAFETY = 0.7;
    private static final double K1 = 5;
    private static final double K2 = 0.8;
    private static final double K3 = 1; 
    
    private final double LIGHT_THRESHOLD = 0.16;
    private static final int LIGHT_FREQUENCE = 7;
    
    public MyRobot (Vector3d position, String name) {
        super(position,name);
        previous_center_observations = new ArrayList<>();
        sonars = RobotFactory.addSonarBeltSensor(this, 12);
        light_center = RobotFactory.addLightSensor(this);
        light_left = RobotFactory.addLightSensorLeft(this);
        light_right = RobotFactory.addLightSensorRight(this); 
    }
    
    @Override
    public void initBehavior() {
        setTranslationalVelocity(0);
        setRotationalVelocity(0);
        switchToState(robotState.Orientation);
    }
    
    private void switchToState(robotState new_state) {
        state = new_state;
        clearObservationValues();
    }
    
    private void clearObservationValues() {
        previous_center_observations.clear();
        previous_left = -1;
        previous_right = -1;
    }
    
    private void forward() {
       setRotationalVelocity(0);
       setTranslationalVelocity(1); 
       
       if (reachedSensorFrequency()) {
            previous_center_observations.add(light_center.getAverageLuminance());
        }
       
       if (reachedGoal()) {
           switchToState(robotState.Stop);
       }
       else if (foundObstacle()) {
           switchToState(robotState.Follow);
       }
       else if (foundLocalMaximum()) {
           switchToState(robotState.Orientation);
       }
    }
    
    private boolean reachedSensorFrequency() {
       return this.getCounter() % LIGHT_FREQUENCE == 0;
    }
    
    private boolean reachedGoal() {
        //return light_center.getAverageLuminance() >= LIGHT_THRESHOLD;
        Point3d robot_pos = new Point3d();
        this.getCoords(robot_pos);
        double distance = robot_pos.distance(new Point3d(5,0,5));
        return distance <= 0.6;
    }
    
    private boolean foundObstacle() {
        double minDist = 2 * SAFETY;
        for (int i=0; i < sonars.getNumSensors(); i++) {          
            if (sonars.hasHit(i) && sonars.getSensorAngle(i) <= Math.PI / 2) {
                minDist = Math.min(minDist,sonars.getMeasurement(i));
            }
        }
        return minDist <= SAFETY;
    }
    
    private boolean foundLocalMaximum() {
        int size = previous_center_observations.size();
        if (size < 3) {
            return false;
        }
        
        float observation1 = previous_center_observations.get(size - 1);
        float observation2 = previous_center_observations.get(size - 2);
        float observation3 = previous_center_observations.get(size - 3);
        return observation1 < observation2 && observation2 > observation3;
    }
    
    private void orientation() {
        setTranslationalVelocity(0); 
        setRotationalVelocity(1); 
       
        if(isAlignedWithLight()) {
            switchToState(robotState.Forward);
        }
       
        if (reachedSensorFrequency()) {
            previous_left = light_left.getAverageLuminance();
            previous_right = light_right.getAverageLuminance();
        }
    }
    
    private boolean isAlignedWithLight() {
        return light_left.getAverageLuminance() < previous_left && light_right.getAverageLuminance() > previous_right;
    }
    
    private void follow() {
        circumventObstacle();
        
        if (reachedSensorFrequency()) {
            previous_center_observations.add(light_center.getAverageLuminance());
        }
        
        if (foundLocalMaximum()) {
            switchToState(robotState.Orientation);
        }      
    }
    
    private void circumventObstacle() {
        int min = 0;
        for (int i = 1; i<sonars.getNumSensors(); i++) {
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min)) {
                min = i;
            }
        }
        
        Point3d p = Tools.getSensedPoint(this, sonars, min);
        double d = p.distance(new Point3d(0,0,0));  
        Vector3d v = new Vector3d(p.z,0,-p.x);
        double phLin = Math.atan2(v.z,v.x);
        double phRot = Math.atan(K3*(d-SAFETY));
        double phRef = Tools.wrapToPi(phLin+phRot); 
        
        setRotationalVelocity(K1*phRef);
        setTranslationalVelocity(K2*Math.cos(phRef));
    }
    
    private void stop() {
       setTranslationalVelocity(0);
       setRotationalVelocity(0); 
    }
    
    @Override
    public void performBehavior() {
        System.out.println(state);
        switch(state) {
            case Forward:
                forward();
                break;
            case Orientation:
                orientation();
                break;
            case Follow:
                follow();
                break;
            case Stop:
                stop();
                break;
        }
        
        //Point3d robot_pos = new Point3d();
        //this.getCoords(robot_pos);
        //double distance = robot_pos.distance(new Point3d(5,0,0));
        //System.out.println("Distance from goal: " + distance);
        
        System.out.println("Center luminance is: " + light_center.getAverageLuminance());
        //System.out.println("Left luminance is: " + light_left.getAverageLuminance());
        //System.out.println("Right luminance is: " + light_right.getAverageLuminance());
    }
}