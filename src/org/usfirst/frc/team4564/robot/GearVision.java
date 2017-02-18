package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The Gear Vision autonomous subsystem.
 * Auto-aligns a robot to the airship gear peg for FIRST Steamworks.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Wataru Nakata
 */
public class GearVision {
	public static GearVision i;
	public static final int MIN_ALIGN_DISTANCE = 35;
	public static final int MAX_ALIGN_DISTANCE = 150;
	
	private Solenoid ringLight;
	private NetworkTable table;
	
	private boolean reached;
	private boolean aligned;
	
	private double forward;
	private double turn;
	private double slide;
	
	public GearVision() {
		i = this;
		table = NetworkTable.getTable("visionTracking");
		ringLight = new Solenoid(Constants.SOL_VISION_RINGLIGHT);
	}
	
	public boolean checkReady() {
		double distance = table.getNumber("distance", 0);
		return distance > MIN_ALIGN_DISTANCE && distance < MAX_ALIGN_DISTANCE;
	}
	
	public void start() {
		Common.debug("start called");
		reset();
		Robot.getDriveTrain().getHeading().setHeadingHold(true);
		ringLight.set(true);
		Common.debug("start end!!!!!!!!!!!!!");
	}
	
	public void reset() {
		//Robot.getDriveTrain().getHeading().setHeadingHold(false);
		reached = false;
		aligned = false;
		turn = 0;
		slide = 0;
		forward = 0;
		ringLight.set(true);
	}

	/**
	 * Calculates drive train motor values to align to the gear peg.
	 */
	public void track() {
		double distance = rawDistance();
		if (distance == 704) {
			forward = 0;
			slide = 0;
			turn = 0;
			return;
		}
		
		//Slide calculation
		double rawSlide = -rawSlide() / 3.0;
		int sign = (rawSlide < 0) ? -1 : 1;
		if(!(rawSlide == 0)) {
			rawSlide = sign * (Math.abs(rawSlide) + 0.23);
		}
		
		//Turn calculation
		double rawTurn = rawTurn();
		if (distance < 70) {
			rawTurn /= -4.504*Math.log(distance) - 18.3;
		}
		rawTurn /= 40.0;
		
		//If minimum distance has been reached, stop to align then SLAM!
		if (distance <= MIN_ALIGN_DISTANCE) {
			reached = true;
		}
		if (reached) {
			if(slide == 0 || aligned) {
				aligned = true;
				forward = -0.6;
				slide = 0;
				turn = -Robot.getDriveTrain().getHeading().turnRate();
			} else {
//				forward = 0;
				slide = -rawSlide;
				forward = 0;
				slide = 0;
				turn = -Robot.getDriveTrain().getHeading().turnRate();
			}
		}
		else {
			double forwardPower = (distance - 28) / 175 + 0.52;
			if(forwardPower > .7 ) {
				forwardPower = .7;
			}
			forwardPower = .58;
			//Adjust heading target based on turn.
			if(distance > 60) {
				Robot.getDriveTrain().getHeading().incrementTargetAngle(rawTurn);
			}
			
			forward = -forwardPower;
			slide = -rawSlide;
			turn = -Robot.getDriveTrain().getHeading().turnRate();
			//forward = 0;
		
		}
	}
	
	/**
	 * Returns the reported distance from the pi vision program.
	 * 
	 * @return double the distance in inches.
	 */
	private double rawDistance() {
		return table.getNumber("distance", 704);
	}
	
	/**
	 * Returns the reported slide offset from the pi vision program.
	 * 
	 * @return double a value from -1.0 to 1.0
	 */
	private double rawSlide() {
		return table.getNumber("slideRate", 704);
	}
	
	/**
	 * Returns the reported turn offset from the pi vision program.
	 * 
	 * @return int a turn value from -5 to 5.
	 */
	private int rawTurn() {
		return (int)table.getNumber("rateTurn", 704);
	}
	
	/**
	 * Returns the calculated forward rate.
	 * 
	 * @return double a speed from -1.0 to 1.0
	 */
	public double forward() {
		return forward;
	}
	
	/**
	 * Returns the calculated slide rate.
	 * 
	 * @return double a speed from -1.0 to 1.0
	 */
	public double slide() {
		return slide;
	}
	
	/**
	 * Returns the calculated turn rate.
	 * 
	 * @return double a speed from -1.0 to 1.0
	 */
	public double turn() {
		return turn;
	}
}
