package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Handles the autonomous routine of the Steamworks 2017 game.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 */
public class Auto {
	private static final int DRIVE_GEAR = 0, ALIGN_GEAR = 1, TRACK_GEAR = 2, DRIVE_HOPPER = 3, SLIDE_HOPPER = 4, 
			DRIVE_BOILER = 5, PIVOT_BOILER = 6, SHOOT = 7;
	private static final int LEFT = 0, CENTER = 1, RIGHT = 2;
	private static final int RED = 0;
	
	private static DriveTrain dt;
	private static NetworkTable autoTable;

	private int startingPosition; //Starting position 0-2, left to right from drive station.
	private int alliance; //0 for red, 1 for blue
	private int action; //0 for gears, 1 for shooting
	private int state; //Current auto state of the robot.
	private long timer;
	
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
		
		if (action == 0) {
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
		else if (action == 1) {
			state = DRIVE_HOPPER;
			shoot();
		}
	}
	
	/**
	 * Controls gear alignment on the left gear platform.
	 */
	public void leftGear() {
		double forwardDistance = (alliance == RED) ? 107.4 : 112.6;
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
		double forwardDistance = 114.3 - 40/*Gear vision distance*/;
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
		double forwardDistance = (alliance == RED) ? 112.6 : 107.4;
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
	
	public void shoot() {
		switch (state) {
			case DRIVE_HOPPER:
				dt.driveDistance(-105);
				state = SLIDE_HOPPER;
				break;
			case SLIDE_HOPPER:
				if (dt.driveComplete()) {
					double slide = (alliance == RED) ? 1.0 : -1.0;
					dt.setDrive(0, 0, slide);
					timer = Common.time() + 3000;
					state = DRIVE_BOILER;
				}
				break;
			case DRIVE_BOILER:
				if (Common.time() >= timer) {
					dt.driveDistance(60);
					state = PIVOT_BOILER;
				}
				break;
			case PIVOT_BOILER:
				if (dt.driveComplete()) {
					dt.manualDrive(0.5, 0.5, 0, 0, 0, 0);
					timer = Common.time() + 1000;
					state = SHOOT;
				}
				break;
			case SHOOT:
				if (Common.time() >= timer) {
					Robot.getThrower().state.fire();
				}
				break;
		}
	}
}
