package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Handles the autonomous routine of the Steamworks 2017 game.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 */
public class Auto {
	private static final int DRIVE_GEAR = 0, ALIGN_GEAR = 1, TRACK_GEAR = 2;
	private static final int LEFT = 0, CENTER = 1, RIGHT = 2;
	private static final int RED = 0;
	private static final int halfRobotWidth = 36/2;
	
	private static DriveTrain dt;
	private static NetworkTable autoTable;

	private int startingPosition; //Starting position 0-2, left to right from drive station.
	private int alliance; //0 for red, 1 for blue
	private int state; //Current auto state of the robot.
	
	public Auto() {
		dt = Robot.getDriveTrain();
		autoTable = NetworkTable.getTable("auto");
	}
	
	public void init() {
		startingPosition = (int)autoTable.getNumber("startingPosition", -1);
		alliance = (int)autoTable.getNumber("alliance", -1);
		startingPosition = RIGHT;
		alliance = RED;
	}
	
	/**
	 * Auto update and state control method.
	 */
	public void auto() {
		switch(startingPosition) {
			case LEFT:
				leftGear();
				break;
			case CENTER: 
				centerGear();
				break;
			case RIGHT: 
				rightGear();
				break;
		}
		
		double forward = 0;
		double turn = 0;
		double slide = 0;
		if (state == TRACK_GEAR) {
			forward = GearVision.i.forward();
			turn = GearVision.i.turn();
			slide = GearVision.i.slide();
		}
		else {
			forward = dt.calcDrive();
			turn = dt.getHeading().turnRate();
		}
		dt.setDrive(forward, turn, slide);
	}
	
	/**
	 * Controls gear alignment on the left gear platform.
	 */
	public void leftGear() {
		double forwardDistance = (alliance == RED) ? 107.4 - halfRobotWidth : 112.6 - halfRobotWidth;
		double turn = 60;
		
		switch (state) {
			case DRIVE_GEAR:
				dt.driveDistance(forwardDistance);
				state = ALIGN_GEAR;
				Common.debug("auto: Driving to gear line.");
				break;
			case ALIGN_GEAR:
				if (dt.driveComplete()) {
					dt.turnTo(turn);
					state = TRACK_GEAR;
					Common.debug("auto: Turning toward gear.");
				}
				break;
			case TRACK_GEAR:
				if (dt.driveComplete()) {
					dt.resetDrive();
					GearVision.i.track();
					Common.debug("auto: Tracking gear.");
				}
				break;
		}
	}
	
	/**
	 * Controls gear alignment on the center gear platform.
	 */
	public void centerGear() {
		double forwardDistance = 114.3 - halfRobotWidth /*Gear vision distance*/;
		switch (state) {
			case DRIVE_GEAR:
				dt.driveDistance(forwardDistance);
				state = TRACK_GEAR;
				Common.debug("auto: Driving to gear line.");
				break;
			case TRACK_GEAR:
				if (dt.driveComplete()) {
					dt.resetDrive();
					GearVision.i.track();
					Common.debug("auto: Tracking gear.");
				}
				break;
		}
	}
	
	/**
	 * Controls gear alignment on the right gear platform.
	 */
	public void rightGear() {
		double forwardDistance = (alliance == RED) ? 112.6 - halfRobotWidth : 107.4 - halfRobotWidth;
		double turn = -60;
		
		switch (state) {
			case DRIVE_GEAR:
				dt.driveDistance(forwardDistance);
				state = ALIGN_GEAR;
				Common.debug("auto: Driving to gear line.");
				break;
			case ALIGN_GEAR:
				if (dt.driveComplete()) {
					dt.turnTo(turn);
					state = TRACK_GEAR;
					Common.debug("auto: Turning toward gear.");
				}
				break;
			case TRACK_GEAR:
				if (dt.driveComplete()) {
					dt.resetDrive();
					GearVision.i.track();
					Common.debug("auto: Tracking gear.");
				}
				break;
		}
	}
}
