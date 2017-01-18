package org.usfirst.frc.team4564.robot;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends XboxController{
	
	private Map<String, Supplier<Boolean>> functionMap = new HashMap<String, Supplier<Boolean>>();
	private Map<String, Boolean> whenMap = new HashMap<String, Boolean>();
	
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
	
	public boolean getPressed(String button) {
		if (functionMap.containsKey(button)) {
			return functionMap.get(button).get();
		}
		return false;
	}
	
	public boolean when(String button) {
		if (!whenMap.containsKey(button)) {
			return false;
		}
		
		if (getPressed(button) && !whenMap.get(button)) {
			whenMap.put(button, true);
			return true;
		}
		else {
			whenMap.put(button, false);
		}
		return false;
	}
	
	public void setupFunctions() {
		functionMap.put("a", this::getAButton);
		whenMap.put("a", false);
		
		functionMap.put("b", this::getBButton);
		whenMap.put("b", false);
		
		functionMap.put("x", this::getXButton);
		whenMap.put("x", false);
		
		functionMap.put("y", this::getYButton);
		whenMap.put("y", false);
		
		functionMap.put("start", this::getStartButton);
		whenMap.put("start", false);
		
		functionMap.put("back", this::getBackButton);
		whenMap.put("back", false);
		
		functionMap.put("dPadUp", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(0 - this.getPOV()) < 45 || Math.abs(360 - this.getPOV()) < 45;
		});
		whenMap.put("dPadUp", false);
		
		functionMap.put("dPadRight", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(90 - this.getPOV()) < 45;
		});
		whenMap.put("dPadRight", false);
		
		functionMap.put("dPadDown", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(180 - this.getPOV()) < 45;
		});
		whenMap.put("dPadDown", false);
		
		functionMap.put("dPadLeft", () -> {
			return (this.getPOV() == -1) ? false : Math.abs(270 - this.getPOV()) < 45;
		});
		whenMap.put("dPadLeft", false);
		
		functionMap.put("leftBumper", () -> {
			return this.getBumper(GenericHID.Hand.kLeft);
		});
		whenMap.put("leftBumper", false);
		
		functionMap.put("rightBumper", () -> {
			return this.getBumper(GenericHID.Hand.kRight);
		});
		whenMap.put("rightBumper", false);
		
		functionMap.put("leftTrigger", () -> {
			return deadzone(this.getLeftTrigger()) > 0;
		});
		whenMap.put("leftTrigger", false);
		
		functionMap.put("rightTrigger", () -> {
			return deadzone(this.getRightTrigger()) > 0;
		});
		whenMap.put("rightTrigger", false);
	}
}
