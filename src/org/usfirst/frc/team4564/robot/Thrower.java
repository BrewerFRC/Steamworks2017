package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import org.usfirst.frc.team4564.robot.Thrower.ThrowerState;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Thrower {
	private PID pid;
	private Spark flywheel;
	private Talon feeder;
	public Encoder encoder;
	public static Xbox j = new Xbox(0);
	private int lastCount;
	private boolean spinning;
	
	//global variables for throwerState class
	public ThrowerState state;
	public int flywheelRPM = 0; //Ideal RPM target for flywheel.
	public int feederRate = 0;  //Ideal RPM for flywheel feeder.
	public static long clearTimer = 0;
	
	public Thrower(double p, double i, double d) {
		pid = new PID(p, i, d, true, "thrower");
		pid.setMax(1.0);
		pid.setMin(-1.0);
		state = new ThrowerState(this);
		flywheel = new Spark(Constants.PWM_FLYWHEEL);
		feeder = new Talon(Constants.PWM_FEEDER_INTAKE);
		flywheel.setInverted(true);
		encoder = new Encoder(Constants.DIO_FLYWHEEL_ENCODER_A,
				Constants.DIO_FLYWHEEL_ENCODER_B,
				false, EncodingType.k4X);
		encoder.setDistancePerPulse(1.0 / Constants.FLYWHEEL_COUNTS_PER_ROT);
	}
	
	public void setPIDOn(boolean on) {
		this.spinning = on;
	}
	
	public PID getPID() {
		return this.pid;
	}
	
	public void setFlywheelSpeed(double speed) {
		flywheel.set(speed);
	}
	
	public void update() {
		//pid.update();
		if (spinning) {
			double speed = pid.calc(getRPM());
			SmartDashboard.putNumber("pidCalc", speed);
			flywheel.set(speed);
		}
	}
	
	public int getRPM() {
		int rpm = (int)(encoder.getRate() * 60);
		SmartDashboard.putNumber("rpm", rpm);
		lastCount = encoder.get();
		return rpm;
	}
	
	public boolean ready() {
		return Math.abs(pid.getTarget() - getRPM()) < Constants.FLYWHEEL_RPM_ALLOWED_ERROR;
	}
	
	//Methods for state cases
	//Sets flywheel RPM
	public void setSpeed(int rpm) {
		pid.setTarget(rpm);
	}
	
	//Sets flywheel feeder to input value
	public void setFeederIntake(double input) {
		feeder.set(input);
	}
	
	public static class ThrowerState {
		public static final int INIT  = 0; //Any initilazation steps
		public static final int READY = 1;  //Ready to begin firing process
		public static final int SPIN_UP = 2;  //Spin up flywheel to set RPM
		public static final int READY_TO_FIRE = 3;  //Flywheel up to input speed
		public static final int FIRE = 4;  //Flywheel feeder intakes ball at feeder rate
		public static final int CLEAR_SHOOTER = 5;  //Stop motors, run feeder back to clear channel.
		
		private Thrower thrower;
		private int currentState;
		private long verificationTimer;
		
		public ThrowerState(Thrower thrower) {
			currentState = INIT;
			verificationTimer = 0;
			this.thrower = thrower;
		}

		public void fire() {
			Common.dashStr("Fire Order Issued:", "Checking flywheel is ready.");
			if(currentState == READY_TO_FIRE) {
				Common.dashStr("Flywheel up to speed:", "Firing");
				currentState = FIRE;
			} else { 
				Common.dashStr("Flywheel not up to speed:", "begining firing process");
				currentState = SPIN_UP;
			}
		}
		
		public void spinUp() {
			currentState = SPIN_UP;
		}
		
		public void stopFiring() {
			currentState = CLEAR_SHOOTER;
		}
		
		public int update() {
			switch(currentState) {
				case READY:
					Common.dashStr("READY:", "Ready to begin firing process");
					break;
				case SPIN_UP:
					Common.dashStr("SPIN_UP:", "Spinning up flywheel");
					thrower.setSpeed(thrower.flywheelRPM);
					if(thrower.ready()) {
						currentState = READY_TO_FIRE;
					} else {
						currentState = SPIN_UP;
					}
					break;
				case READY_TO_FIRE:
					Common.dashStr("READY_TO_FIRE:", "Flywheel up to speed");
					break;
				case FIRE:
					Common.dashStr("FIRE", "Activating flywheel feeder, firing ball");
					thrower.setFeederIntake(thrower.feederRate);
					break;
				case CLEAR_SHOOTER:
					Common.dashStr("CLEAR_SHOOTER:", "Clearing channel of balls");
					thrower.setSpeed(0);
					thrower.setFeederIntake(-1);
					clearTimer = Common.time() + 500;
					if(Common.time() >= clearTimer) {
						currentState = READY;
					}
					break;
			}
			return currentState;
		}
	}
}
