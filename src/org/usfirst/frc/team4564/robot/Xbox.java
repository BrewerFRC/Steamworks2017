package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.Joystick;


public class Xbox extends Joystick{

	public Xbox(int port) {
		super(port);
	}
	
	public boolean getAButton() {
		return false;
	}
}
