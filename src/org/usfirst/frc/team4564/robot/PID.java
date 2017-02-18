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
	
	/**
	 * Instantiate a new PID controller with the defined terms.
	 * 
	 * @param p the p scaler.
	 * @param i the integral scaler.
	 * @param d the derivative scaler.
	 * @param forward whether or not to cummulate calculations.
	 * @param name the name of the pid object, used for NetworkTables posting.
	 */
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
	
	/**
	 * Update the scaler terms with the latest NetworkTables values.
	 */
	public void update() {
		this.p = SmartDashboard.getNumber(this.name + "P", this.p);
		this.i = SmartDashboard.getNumber(this.name + "I", this.i);
		this.d = SmartDashboard.getNumber(this.name + "D", this.d);
	}
	
	/**
	 * Sets the minimum and maximum output values of the PID calculation.
	 * 
	 * @param min the minimum output value.
	 * @param max the maximum output value.
	 */
	public void setOutputLimits(double min, double max){
		this.Outmin = min;
		this.Outmax = max;
	}
	
	/**
	 * The minimum distance from zero the output calculation can be.
	 * 
	 * @param min the minimum absolute value of the output calculation.
	 */
	public void setMin(double min) {
		this.min = min;
	}
	
	/**
	 * Sets the starting output of a forward PID.
	 * 
	 * @param output the starting output value.
	 */
	public void setStartingOutput(double output) {
		this.output = output;
	}
	
	/**
	 * Returns the target value of the PID controller.
	 * 
	 * @return double the current target value.
	 */
	public double getTarget() {
		return this.target;
	}
	
	/**
	 * Sets the current target value of the PID controller.
	 * 
	 * @param target the new target value.
	 */
	public void setTarget(double target) {
		this.target = target;
	}
	
	/**
	 * Returns the P scaler.
	 * 
	 * @return double the P scaler.
	 */
	public double getP() {
		return this.p;
	}
	
	/**
	 * Sets the P scaler.
	 * 
	 * @param p the new P scaler.
	 */
	public void setP(double p) {
		this.p = p;
	}
	
	/**
	 * Returns the integral scaler.
	 * 
	 * @return double the integral scaler.
	 */
	public double getI() {
		return this.i;
	}
	
	/**
	 * Sets the integral scaler.
	 * 
	 * @param i the new integral scaler.
	 */
	public void setI(double i) {
		this.i = i;
	}
	
	/**
	 * Returns the derivative scaler.
	 * 
	 * @return double the derivative scaler.
	 */
	public double getD() {
		return this.d;
	}
	
	/**
	 * Sets the derivative scaler.
	 * 
	 * @param d the new derivative scaler.
	 */
	public void setD(double d) {
		this.d = d;
	}
	
	/**
	 * Resets the cummulative values of the PID.
	 */
	public void reset() {
		sumError = 0;
		previousError = 0;
		output = 0;
	}
	
	/**
	 * Returns the last calculation.
	 * 
	 * @return double the previous calculation.
	 */
	public double getLastCalc() {
		return this.lastCalc;
	}
	
	/**
	 * Calculates a new output value based on an input and a target.
	 * 
	 * @param input the input to weigh against the target.
	 * @return double the output calculation.
	 */
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
		if(output > 0) 
			sign = 1;
		output = Math.abs(output)+ min;
		output *= sign;
		output = Math.min(Math.max(output, Outmin), Outmax);
		
		//Apply necessary forward PID cummulation.
		if (forward) {
			this.output += output;
		}
		else {
			this.output = output;
		}
		lastCalc = this.output;
		
		return this.output;
	}
}