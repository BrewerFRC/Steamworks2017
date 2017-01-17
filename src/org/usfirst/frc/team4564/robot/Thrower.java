package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Thrower {
	private PID pid;
	private Spark flywheel;
	public Encoder encoder;
	public static Xbox j = new Xbox(0);
	private int lastCount;
	private boolean spinning;
	
	public ThrowerState state;
	public int flywheelRPM = 0; //Ideal RPM target for flywheel.
	public int feederRate = 0;  //Ideal RPM for flywheel feeder.
	public static long ejectTimer = 0;
	
	public Thrower(double p, double i, double d) {
		pid = new PID(p, i, d, true, "thrower");
		pid.setMax(1.0);
		pid.setMin(-1.0);
		flywheel = new Spark(Constants.PWM_FLYWHEEL);
		flywheel.setInverted(true);
		encoder = new Encoder(Constants.DIO_FLYWHEEL_ENCODER_A,
				Constants.DIO_FLYWHEEL_ENCODER_B,
				false, EncodingType.k4X);
		encoder.setDistancePerPulse(1.0 / Constants.FLYWHEEL_COUNTS_PER_ROT);
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
	
	public void update() {
		//pid.update();
		if (spinning) {
			double speed = pid.calc(getRPM());
			SmartDashboard.putNumber("pidCalc", speed);
			flywheel.set(speed);
		}
		else {
			flywheel.set(0);
		}
	}
	
	public int getRPM() {
		int rpm = (int)(encoder.getRate() * 60);
		SmartDashboard.putNumber("rpm", rpm);
		lastCount = encoder.get();
		return rpm;
	}
	
	public boolean ready() {
		//EVAN, please put in your logic for determining if target set in PID has been reached (with deadzone), thanks, -Jacob
		return true;
		
	}
	
	//Methods for state cases
	//Sets flywheel RPM
	public void setSpeed(int rpm) {
		pid.setTarget(rpm);
	}
	
	//Spins up flywheel to determined RPM stored in flywheelRPM variable
	public void spinUp() {
		this.setSpeed(flywheelRPM);
	}
	
	//Sets flywheel feeder to input value
	public double setFeederIntake(double input) {
		//EVAN, please put in how you want to set motor power for flywheel intake motor, thanks, -Jacob
		return (1.0);
	}
	
	public void stopFlywheel() {
		this.setSpeed(0);
	}
	
	public void feederEject() {
		this.setFeederIntake(-1.0);
	}
	
	public static class ThrowerState {
		public static final int INIT  = 0; //Any initilazation steps
		public static final int READY = 1;  //Ready to begin firing process
		public static final int SPIN_UP = 2;  //Spin up flywheel to set RPM
		public static final int READY_TO_FIRE = 3;  //Flywheel up to input speed
		public static final int FIRING = 4;  //Flywheel feeder intakes ball at feeder rate
		public static final int CLEAR_SHOOTER = 5;  //Stop motors, run feeder back to clear channel.
		
		private Thrower thrower;
		private int currentState;
		private long verificationTimer;
		
		public ThrowerState(Thrower thrower) {
			currentState = INIT;
			verificationTimer = 0;
			this.thrower = thrower;
		}
		
		public void init() {
			currentState = READY;
		}

		//Spin the flywheel up to speed to fire
		public void prepForFire() {
			if(currentState == SPIN_UP) {
				thrower.spinUp();
				if(thrower.ready()) {
					currentState = READY_TO_FIRE;
				} else {
					currentState = SPIN_UP;
				}
			}
		}
		
		//Make sure the operator is still asking for a ball to be fired.
		public void verifyFireOrder() {
			if(currentState == READY_TO_FIRE && j.getBumper(GenericHID.Hand.kRight) == true) {
				currentState = FIRING;
			} else { 
				if(verificationTimer < 50) {
					verificationTimer += 1;
					verifyFireOrder();
				} else {
					currentState = CLEAR_SHOOTER;
					verificationTimer = 0;
				}
			}
		}
		
		//Fire balls
		public void fire() { 
			thrower.setFeederIntake(thrower.feederRate);
			verifyFireOrder();
		}
		
		public void clear() {
			if(currentState == CLEAR_SHOOTER) {
				thrower.stopFlywheel();
				thrower.feederEject();
				ejectTimer = Common.time() + 500;
				if(Common.time() >= ejectTimer) {
					currentState = READY;
				}
			}
		}
		
		public int update() {
			switch(currentState) {
				case READY:
					Common.dashStr("READY:", "Ready to begin firing process");
					break;
				case SPIN_UP:
					Common.dashStr("SPIN_UP:", "Spinning up flywheel");
					this.prepForFire();
					break;
				case READY_TO_FIRE:
					Common.dashStr("READY_TO_FIRE:", "Flywheel up to speed");
					verifyFireOrder();
					break;
				case FIRING:
					Common.dashStr("FIRING", "Activating flywheel feeder, firing ball");
					fire();
					break;
				case CLEAR_SHOOTER:
					Common.dashStr("CLEAR:", "Clearing channel of balls");
					clear();
					break;
			}
			return currentState;
		}
	}
}
