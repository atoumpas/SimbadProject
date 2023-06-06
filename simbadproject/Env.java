package simbadproject;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import simbad.sim.Box;
import simbad.sim.CherryAgent;
import simbad.sim.EnvironmentDescription;

public class Env extends EnvironmentDescription {
    private final Vector3d target_position;
    private final Vector3d robot_position;
    private final float light_height;
    
    Env(){
        target_position = new Vector3d(5, 0, 5);
        robot_position = new Vector3d(0, 0, 0);
        light_height = 5;
        
        setBaseAgents();
        setObstacles();
        setLight();
    }
    
    private void setBaseAgents() {
        add(new CherryAgent(target_position,"target",0.1f));
        add(new MyRobot(robot_position, "my robot"));
    }
    
    private void setObstacles() {
        add(new Box(new Vector3d(0,0,3), new Vector3f(3, 1,5),this));
        add(new Box(new Vector3d(6,0,1), new Vector3f(4, 1,3),this));
    }
    
    private void setLight() {
        // Remove other light sources
        this.ambientLightColor = this.black;
        this.backgroundColor = this.black;
        
        Vector3d light_position = target_position;
        light_position.y = light_height;
        this.light1Position = light_position;
    }
}