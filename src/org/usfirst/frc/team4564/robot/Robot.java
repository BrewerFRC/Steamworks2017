package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	public static NetworkTable table;
	public Thrower thrower;
	
    public Robot() {
    	table = NetworkTable.getTable("table");
    	thrower = new Thrower(0.1, 0, 0);
    }
    
    public void robotInit() {
    	
    }

    public void autonomous() {
    	
    }
    public void operatorControl() {
       long time;
       
       while(true) {
    	   time = Common.time();
    	   
    	   //Do stuff
    	   thrower.update();
    	   SmartDashboard.putNumber("encoder", thrower.encoder.get());
    	   
    	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
       }
    }
    public void test() {
    }
    
    public void disabled() {
    	long time;
        
        while(true) {
     	   time = Common.time();
     	   
     	   SmartDashboard.putNumber("encoder", thrower.encoder.get());
     	   
     	   Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
        }
    }
}
