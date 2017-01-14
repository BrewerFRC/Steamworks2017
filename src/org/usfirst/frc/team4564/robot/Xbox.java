package org.usfirst.frc.team4564.robot;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends XboxController{
	
	private Map<String, Supplier<Boolean>> functionWhen;

	public Xbox(int port) {
		super(port);
		setupFunctions();
	}
	
	//Deadzone
	public double deadzone(double input) {
		if (Math.abs(input) < .2) {
			return(0);
		} else {
			return(input);
		}
	}
	
	/*
	//Prev Function
	public void prev(Function()) {
		return (prev + input);
	}
	
	//Rising edge function
	public boolean when(Function) {
		if(FunctionValue()) {
			if(prev(Function())) {
				return false;
			} else {
				prev(Function()) = true;
				return true;
			}
		} else {
			prev(Function()) = false;
			return false;
		}
	}
	*/
	
	//Triggers
	public double getRightTrigger() {
		return deadzone(getTriggerAxis(GenericHID.Hand.kRight));
	}
	
	public double getLeftTrigger() {
		return deadzone(getTriggerAxis(GenericHID.Hand.kLeft));
	}
	
	public void setupFunctions() {
		
	}
}
