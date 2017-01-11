package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Thrower {
	private PID pid;
	private Spark flywheel;
	
	public Encoder encoder;
	private int lastCount;
	
	private boolean spinning;
	
	public Thrower(double p, double i, double d) {
		pid = new PID(p, i, d, true, "thrower");
		flywheel = new Spark(Constants.PWM_FLYWHEEL);
		encoder = new Encoder(Constants.DIO_FLYWHEEL_ENCODER_A,
				Constants.DIO_FLYWHEEL_ENCODER_B,
				false, EncodingType.k4X);
	}
	
	public void on() {
		this.spinning = true;
	}
	
	public void off() {
		this.spinning = false;
	}
	
	public boolean isOn() {
		return this.spinning;
	}
	
	public void setSpeed(int rpm) {
		pid.setTarget(rpm);
	}
	
	public void update() {
		pid.update();
		if (spinning) {
			flywheel.set(pid.calc(getRPM()));
		}
		else {
			flywheel.set(0);
		}
	}
	
	public int getRPM() {
		int rpm = (encoder.get() - lastCount) / Constants.FLYWHEEL_COUNTS_PER_ROT;
		SmartDashboard.putNumber("rpm", rpm);
		lastCount = encoder.get();
		return rpm;
	}
}
