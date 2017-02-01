package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {
	//Terms
	private String name;
	private double p;
	private double i;
	private double d;
	
	/* 
	 * Whether or not to cummulate values over time.
	 * True = cummulate, False = raw return value
	 */
	private boolean forward;
	private double target;
	private double Outmin = -1;
	private double Outmax = 1;
	private double min;
	private long deltaTime;
	private double previousError;
	private double sumError;
	private double output;
	private double lastCalc;
	
	public PID(double p, double i, double d, boolean forward, String name) {
		this.name = name;
		this.p = p;
		this.i = i;
		this.d = d;
		this.forward = forward;
		this.deltaTime = (long)(1.0/Constants.REFRESH_RATE*1000);
		
		SmartDashboard.putNumber(this.name + "P", this.p);
		SmartDashboard.putNumber(this.name + "I", this.i);
		SmartDashboard.putNumber(this.name + "D", this.d);
	}
	
	public void update() {
		this.p = SmartDashboard.getNumber(this.name + "P", this.p);
		this.i = SmartDashboard.getNumber(this.name + "I", this.i);
		this.d = SmartDashboard.getNumber(this.name + "D", this.d);
	}
	
	public void setOutputLimits(double min, double max){
		this.Outmin = min;
		this.Outmax = max;
	}
	public void setMin(double min) {
		this.min = min;
	}
	public double getTarget() {
		return this.target;
	}
	public void setTarget(double target) {
		this.target = target;
	}
	public double getP() {
		return this.p;
	}
	public void setP(double p) {
		this.p = p;
	}
	public double getI() {
		return this.i;
	}
	public void setI(double i) {
		this.i = i;
	}
	public double getD() {
		return this.d;
	}
	public void setD(double d) {
		this.d = d;
	}
	public void reset() {
		sumError = 0;
		previousError = 0;
	}
	public double getLastCalc() {
		return this.lastCalc;
	}
	
	public double calc(double input) {
		
		//Integral calculation
		double error = input - target;
		sumError += error * deltaTime;
		int sign = -1;
		
		//Derivative calculation
		double derivative = (error - previousError) / deltaTime;
		previousError = error;
		
		//Calculate output
		double output = p*error + i*sumError + d*derivative;
		if(output>0) 
			sign = 1;
		output = Math.abs(output)+ min;
		output *= sign;
		output = Math.min(Math.max(output, Outmin), Outmax);
		

		if (forward) {
			this.output += output;
		}
		else {
			this.output = output;
		}
		lastCalc = this.output;
		//Common.debug(this.name + "PID_OUT" + this.output );
		return this.output;
	}
}
