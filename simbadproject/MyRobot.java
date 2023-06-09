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
    private final ArrayList<Float> previous_observations;
    private final RangeSensorBelt sonars;  
    private final LightSensor light_center;
    private final LightSensor light_left;
    private final LightSensor light_right;
    
    private static final double SAFETY = 0.7;
    private static final double K1 = 5;
    private static final double K2 = 0.8;
    private static final double K3 = 1; 
    
    private final double LIGHT_THRESHOLD_CENTER = 0.10;
    private final double LIGHT_THRESHOLD_SIDE = 0.04;
    private static final int LIGHT_FREQUENCE = 7;
    
    public MyRobot (Vector3d position, String name) {
        super(position,name);
        previous_observations = new ArrayList<>();
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
        previous_observations.clear();
    }
    
    private void forward() {
       setRotationalVelocity(0);
       setTranslationalVelocity(1); 
       
       if (reachedSensorFrequency()) {
            previous_observations.add(light_center.getAverageLuminance());
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
        return light_center.getAverageLuminance() >= LIGHT_THRESHOLD_CENTER &&
               light_left.getAverageLuminance() <= LIGHT_THRESHOLD_SIDE &&
               light_right.getAverageLuminance() <= LIGHT_THRESHOLD_SIDE;        
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
        int size = previous_observations.size();
        if (size < 3) {
            return false;
        }
        
        double observation1 = previous_observations.get(size - 1);
        double observation2 = previous_observations.get(size - 2);
        double observation3 = previous_observations.get(size - 3);
        return observation1 < observation2 && observation2 > observation3;
    }
    
    private void orientation() {
        setTranslationalVelocity(0); 
        double new_rotational_velocity;
        if (lightIsLeft()) {
            new_rotational_velocity = 0.5;
        }
        else {
            new_rotational_velocity = -0.5;
        }
        
        if (isSwitchingRotation(new_rotational_velocity)) {
            // Make more sudden turn to better align with light in last clock hit, before switching state
            new_rotational_velocity *= 10;
            switchToState(robotState.Forward);
        }
        
        setRotationalVelocity(new_rotational_velocity);
    }
    
    private boolean lightIsLeft() {
        return light_left.getAverageLuminance() > light_right.getAverageLuminance();
    }
    
    private boolean isSwitchingRotation(double new_velocity) {
        return new_velocity == -this.getRotationalVelocity();
    }
    
    private void follow() {
        circumventObstacle();
        
        if (reachedSensorFrequency()) {
            previous_observations.add(light_center.getAverageLuminance());
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
        System.out.println("Current state: " + state);
        System.out.println(light_center.getAverageLuminance());
        System.out.println(light_left.getAverageLuminance());
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
    }
}