package simbadproject;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import simbad.sim.Box;
import simbad.sim.CherryAgent;
import simbad.sim.EnvironmentDescription;

public class Env extends EnvironmentDescription {
    private Vector3d target_position;
    private Vector3d robot_position;
    private final float light_height = 2;
    private final int environment;
    
    Env(){
        //Select environment from 1 to 3
        environment = 1;

        setBaseAgents();
        setObstacles();
        setLight();
    }
    
    private void setBaseAgents() {
        setAgentCoordinates();
        add(new CherryAgent(target_position,"target",0.1f));
        add(new MyRobot(robot_position, "my robot"));
    }
    
    private void setAgentCoordinates() {
        Vector3d target_vector, robot_vector;
        switch(environment) {
            case 2:
                target_vector = new Vector3d(-5, 0, 5);
                robot_vector = new Vector3d(0, 0, 0);
                break;
            case 3:
                target_vector = new Vector3d(-3, 0, 4);
                robot_vector = new Vector3d(0, 0, -5);
                break;
            default:
                target_vector = new Vector3d(3, 0, 0);
                robot_vector = new Vector3d(-7, 0, 2);
                break;
        }
        target_position = target_vector;
        robot_position = robot_vector;
    }
    
    private void setObstacles() {
        switch(environment) {
            case 2:
                add(new Box(new Vector3d(0,0,3), new Vector3f(3, 1,5),this));
                add(new Box(new Vector3d(6,0,1), new Vector3f(4, 1,3),this));
                break;
            case 3:
                add(new Box(new Vector3d(0,0,3), new Vector3f(2, 1,6),this));
                Box box2 = new Box(new Vector3d(-2,0,-1), new Vector3f(2, 1,10),this);
                box2.rotate90(1);
                add(box2);
                break;
            default:
                add(new Box(new Vector3d(-4,0,0), new Vector3f(2, 1,6),this));
                break;
        }
    }
    
    private void setLight() {
        // Remove other light sources
        this.ambientLightColor = this.black;
        this.backgroundColor = this.black;
        this.light2IsOn = false;
        
        this.light1IsOn = true;
        Vector3d light_position = target_position;
        light_position.y = light_height;
        this.light1Position = light_position;
    }
}