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
    		if(j.when("rightTrigger")) {
    			Common.debug("Testing Triggers");
    			if(prevRightTrigger == false) {
    				Common.debug("Turning on Fire");
    				thrower.state.fire();
    				prevRightTrigger = true;
    			} else {
    				Common.debug("Turning off Fire");
    				thrower.state.stopFiring();
    				thrower.clearTimer = Common.time() + 500;
    				prevRightTrigger = false;
    			}
    		}
    		
    		if (j.getXButton()) {
    			thrower.setPIDOn(false);
    			thrower.setFlywheelSpeed(flywheelSpeed);
    		}
    		else {
    			thrower.setPIDOn(true);
    			int direction = 0;
    			if (j.when("dPadUp")) {
    				System.out.println("dPadUp");
    				direction = 1;
    			}
    			else if (j.when("dPadDown")) {
    				System.out.println("dPadDown");
    				direction = -1;
    			}
    			PID pid = thrower.getPID();
    			if (j.getYButton()) {
        			pid.setP(pid.getP() + direction*0.000002);
        			SmartDashboard.putNumber("pidP", pid.getP());
        		}
        		else if (j.getBButton()) {
        			pid.setI(pid.getI() + direction*0.000002);
        			SmartDashboard.putNumber("pidI", pid.getI());
        		}
        		else if (j.getAButton()) {
        			pid.setD(pid.getD() + direction*0.001);
        			SmartDashboard.putNumber("pidD", pid.getD());
        		}
    		}
    		
    		
    		//End of Xbox tests
    		SmartDashboard.putNumber("encoder", thrower.encoder.get());
    	   thrower.state.update();
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
