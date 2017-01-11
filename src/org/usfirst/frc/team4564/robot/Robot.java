package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	public static XboxController j = new XboxController(0);
	
    public Robot() {
    }
    
    public void robotInit() {

    }

    public void autonomous() {
    	
    }
    public void operatorControl() {
    	while (isOperatorControl() && isEnabled()) {
    		if (j.getAButton()) {
    			SmartDashboard.putBoolean("A Button", j.getAButton());
    		}
    	}
    }
    public void test() {
    }
}
