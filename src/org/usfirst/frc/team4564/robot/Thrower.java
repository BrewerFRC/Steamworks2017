package org.usfirst.frc.team4564.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles the functions and state of the thrower subsystem.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Jacob Cote
 * @author Wataru Nakata
 */
public class Thrower {
	private CANTalon flywheel0, flywheel1;
	private Spark feeder;
	private Spark intake;
	private PID flywheelPID;
	private Servo flipper;
	public static Xbox j = new Xbox(0);
	
	public ThrowerState state;
	private static int flywheelRPM = 2900; //Ideal RPM target for flywheel.
	private static  double feederRate = -0.28;  //Ideal RPM for flywheel feeder.
	private static final double intakeRate = 1.0;
	private static final double P = 1.5e-5;
	private static final double I = 5.0e-11;
	private static final double D = 0.006;
	
	private boolean engaged = false;
	public int intakeState; //-1 puke, 0 off, 1 intake
	private boolean shooting;
	private boolean puke;
	
	/**
	 * Instantiates a new Thrower subsystem with the defined PID values.
	 * 
	 * @param p the P scaler.
	 * @param i the integral scaler.
	 * @param d the derivative scaler.
	 */
	public Thrower() {
		state = new ThrowerState(this);
		
		//Instantiate motor controllers.
		flywheel0 = new CANTalon(Constants.CANID_THROWER_FLYWHEEL_0);
		flywheel0.changeControlMode(TalonControlMode.PercentVbus);
		flywheel1 = new CANTalon(Constants.CANID_THROWER_FLYWHEEL_1);
		flywheel1.changeControlMode(TalonControlMode.PercentVbus);
		flywheel1.configEncoderCodesPerRev(1024);
		
		feeder = new Spark(Constants.PWM_THROWER_INTERNAL_INTAKE);
		intake = new Spark(Constants.PWM_THROWER_INTAKE);
		flywheelPID = new PID(P, I, D, true, "flywheel");
		
		flipper = new Servo(Constants.SERVO);
		SmartDashboard.putNumber("Target Flywheel RPM", flywheelRPM);
		SmartDashboard.getNumber("feederRate", feederRate);
	}
	
	/**
	 * Returns the current flywheel speed in rotations per minute.
	 * 
	 * @return the speed in RPM.
	 */
	public double getRPM() {
		return flywheel1.getEncVelocity()/10240.0*1500;
	}
	public void deployFlipper() {
			flipper.set(1);
	}
	
	public void retractFlipper() {
			flipper.set(-1);
	}
	
	/**
	 * Whether or not the flywheel is spun up and ready to shoot.
	 * 
	 * @return boolean whether or not the thrower is ready.
	 */
	public boolean ready() {
		return Math.abs(getRPM() - flywheelPID.getTarget()) < Constants.FLYWHEEL_RPM_ALLOWED_ERROR;
	}
	
	/**
	 * Sets the speed of the flywheel.
	 * 
	 * @param rpm the speed in rotations per minute.
	 */
	public void setSpeed(int rpm) {
		engaged = (rpm != 0);
		flywheelPID.setTarget(rpm);
	}
	
	/**
	 * Sets the motor power for the feeder intake.
	 * 
	 * @param input a motor power from -1.0 to 1.0.
	 */
	public void setFeederIntake(double input) {
		feeder.set(input);
	}
	
	/**
	 * Runs the external intake.
	 */
	public void setIntake(double speed) {
		intake.set(speed);
	}
	
	/**
	 * Turns on intake.
	 */
	public void intakeOn() {
		puke = false;
		intakeState = 1;
		Common.debug("Turing On Intake");
	}
	
	/**
	 * Turns intake backward.
	 */
	public void intakeBackward() {
		puke = true;
		intakeState = -1;
	}
	
	/**
	 * Shuts off the external intake.
	 */
	public void intakeOff() {
		puke = false;
		intakeState = 0;
		Common.debug("Turing Off Intake");
	}
	
	/**
	 * Toggles the external intake.
	 */
	public void toggleIntake() {
		if (intake.get() > 0) {
			intakeOff();
		}
		else {
			intakeOn();
		}
	}
	
	/**
	 * Updates the PID coefficients and PID calc.
	 */
	public void update() {
		flywheelPID.update();
		feederRate =  SmartDashboard.getNumber("feederRate", -0.28);
		flywheelRPM = (int) SmartDashboard.getNumber("Target Flywheel RPM", flywheelRPM);
		
		intake.set(intakeState * intakeRate);
		 if (!shooting) {
			 if (puke){
				 feeder.set(intakeState * feederRate);
			 }
			 else {
				 feeder.set(-intakeState * feederRate);
			 }
		}
		
		if (engaged) {
			double pidCalc = flywheelPID.calc(getRPM());
			SmartDashboard.putNumber("flywheelPIDOutput", pidCalc);
			flywheel0.set(pidCalc);
			flywheel1.set(pidCalc);
		}
		else {
			flywheel0.set(0);
			flywheel1.set(0);
			flywheelPID.reset();
		}
	}
	
	/**
	 * Handles the state progression of the shooting process.
	 * 
	 * @author Brewer FIRST Robotics Team 4564
	 * @author Jacob Cote
	 * @author Evan McCoy
	 */
	public class ThrowerState {
		public static final int INIT  = 0; //Any initilazation steps
		public static final int READY = 1;  //Ready to begin firing process
		public static final int SPIN_UP = 2;  //Spin up flywheel to set RPM
		public static final int READY_TO_FIRE = 3;  //Flywheel up to input speed
		public static final int FIRE = 4;  //Flywheel feeder intakes ball at feeder rate
		public static final int CLEAR_SHOOTER = 5;  //Stop motors, run feeder back to clear channel.
		public static final int DEPLOY_FLIPPER = 6;  //deploy flipper with 3 seconds delay
		
		private Thrower thrower;
		public int currentState;
		public long clearTimer,flipperDelay = 0;
		public boolean fire = false;
		public boolean flipperCheck;
		
		public ThrowerState(Thrower thrower) {
			currentState = INIT;
			this.thrower = thrower;
		}

		/**
		 * Sets the thrower to firing state if prepared, otherwise the thrower continues to spin up.
		 */
		public void fire() {
			fire = true;
			flipper.set(-1);
			Common.debug("Fire Order Issued: Checking flywheel is ready.");
			if(currentState == READY_TO_FIRE) {
				Common.debug("Flywheel up to speed: Firing");
				currentState = FIRE;
			} else { 
				Common.debug("Flywheel not up to speed: begining firing process");
				currentState = SPIN_UP;
			}
		}
		
		/**
		 * Sets the thrower to spin up state.
		 */
		public void spinUp() {
			flywheelPID.reset();
			currentState = SPIN_UP;
		}
		
		/**
		 * Sets the shooter to clearing state to remove balls and stop the flywheel.
		 */
		public void stopFiring() {
			currentState = CLEAR_SHOOTER;
		}
		
		/**
		 * Updates the thrower through state progression.
		 * 
		 * @return int the current state of the thrower.
		 */
		public int update() {
			switch(currentState) {
				case READY:
					break;
					
				case SPIN_UP:
					flipperCheck = false;
					Common.debug("SPIN_UP: Spinning up flywheel");
					thrower.setSpeed(flywheelRPM);
					thrower.intakeOn();
					if(thrower.ready() && fire) {
						currentState = READY_TO_FIRE;
					} else {
						currentState = SPIN_UP;
					}
					break;
					
				case READY_TO_FIRE:
					Common.debug("READY_TO_FIRE: Flywheel up to speed");
					currentState = FIRE;
					break;
					
				case FIRE:
					Common.debug("FIRE: Activating flywheel feeder, firing ball");
					thrower.shooting = true;
					thrower.setFeederIntake(feederRate);
					clearTimer = Common.time() + 500;
		
					break;
				case CLEAR_SHOOTER:
					fire = false;
					Common.debug("CLEAR_SHOOTER: Clearing channel of balls");
					thrower.intakeOff();
					thrower.setSpeed(0);
					//flipper.set(1);
					thrower.setFeederIntake(1);
					if(Common.time() >= clearTimer) {
						thrower.setFeederIntake(0);
						flipperDelay = Common.time() + 3000;
						currentState = DEPLOY_FLIPPER;
						thrower.shooting = false;
					} 
					break;
				case DEPLOY_FLIPPER:
					if(Common.time() > flipperDelay){
						deployFlipper();
						currentState = READY;
					}
					flipperCheck = true;
			}
			return currentState;
		}
	}
}
