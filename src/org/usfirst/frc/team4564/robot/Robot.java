package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

public class Robot extends SampleRobot {
	private Xbox j ;
	public static NetworkTable table;
	private static Thrower thrower;
	private boolean prevRightTrigger;
	
    public Robot() {
    	j = new Xbox(0);
    	prevRightTrigger = false;
    	table = NetworkTable.getTable("table");
    	thrower = new Thrower(0.000012, 0, 0.008);
    }
    
    public void robotInit() {

    }

    public void autonomous() {
    	
    }
    
    public void operatorControl() {
    	long time;
	    //thrower.setSpeed(3300);
	    thrower.setPIDOn(true);
	    double flywheelSpeed = 0.5;
    	while (isEnabled() && isOperatorControl()) {
    		time = Common.time();
    		Common.debug("Right Joystick:"+ j.when("rightTrigger"));
    		if(j.when("rightTrigger")) {
    			if(prevRightTrigger == false) {
    				Common.dashStr("Turning on Fire", "");
    				thrower.state.fire();
    				prevRightTrigger = true;
    			} else {
    				Common.dashStr("Turning off fire", "");
    				thrower.state.stopFiring();
    				prevRightTrigger = false;
    			}
    		}
    		
    		if (j.getXButton()) {
    			thrower.setPIDOn(false);
    			thrower.setFlywheelSpeed(flywheelSpeed);
    		}
    		else {
    			int direction = 0;
    			if (j.when("dPadUp")) {
    				System.out.println("dPadUp");
    				//direction = 1;
    			}
    			else if (j.when("dPadDown")) {
    				System.out.println("dPadDown");
    				//direction = -1;
    			}
    			PID pid = thrower.getPID();
    			if (j.getYButton()) {
        			thrower.setPIDOn(true);
        			pid.setP(pid.getP() + direction*0.000002);
        		}
        		else if (j.getBButton()) {
        			thrower.setPIDOn(true);
        			pid.setI(pid.getI() + direction*0.000002);
        		}
        		else if (j.getAButton()) {
        			thrower.setPIDOn(true);
        			pid.setD(pid.getD() + direction*0.001);
        		}
    		}
    		
    		
    		//End of Xbox tests
    		SmartDashboard.putNumber("encoder", thrower.encoder.get());
    	   thrower.update();
    	   Timer.delay(1.0/50);
    	   //Timer.delay((1000/Constants.REFRESH_RATE - (Common.time() - time)) / 1000);
    	}
    }

    public void test() {
    	
    }
    
    public void disabled() {
    }
}
