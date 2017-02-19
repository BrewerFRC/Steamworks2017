package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Handles the autonomous routine of the Steamworks 2017 game.
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Wataru Nakata
 * @author Jacob Cote
 */
public class Auto {
	private static final int DRIVE_TO_GEAR = 1, TURN_TO_GEAR = 2, ALIGN = 3,GEAR_VISION = 4, 
			DRIVE_HOPPER = 0, SLIDE_HOPPER = 1, DRIVE_BOILER = 2, PIVOT_BOILER = 3, SHOOT = 4;
	private static final int LEFT = 0, CENTER = 1, RIGHT = 2;
	private static final int ACTION_GEAR = 0, ACTION_BOILER = 1;
	private static final int RED = 0;
	private static final int halfRobotWidth = 36/2;
	
	private static DriveTrain dt;
	private static NetworkTable autoTable;

	private int startingPosition; //Starting position 0-2, left to right from drive station.
	private int alliance; //0 for red, 1 for blue
	private int action; //0 for gears, 1 for shooting
	private int state; //Current auto state of the robot.
	private long timer;
	
	private double distance;
	private double turn;
	
	public Auto(DriveTrain drivetrain) {
		dt = drivetrain;
		autoTable = NetworkTable.getTable("auto");
	}
	
	public void init() {
		state = DRIVE_TO_GEAR;
		dt.getHeading().setHeadingHold(true);
//		startingPosition = (int)autoTable.getNumber("startingPosition", -1);
//		alliance = (int)autoTable.getNumber("alliance", -1);
//		action = (int)autoTable.getNumber("action", -1);
		startingPosition = LEFT;
		alliance = RED;
		action = ACTION_GEAR;
		
		Common.debug("AUTO:CALC");
		
		switch(startingPosition) {
		
			case RIGHT:
				if(alliance == RED) {
					distance = 112.6 - halfRobotWidth;
					turn = -60;
				} else {
					distance = 107.4 - halfRobotWidth;
					turn = -60;
				}
				break;
				
			case LEFT:
				if(alliance == RED) {
					distance = 107.4 - halfRobotWidth;
					turn = 60;
				} else {
					distance = 112.6 - halfRobotWidth;
					turn = 60;
				}
				break;
				
			case CENTER:
				distance = 0;
				turn = 0;
				break;
				
		}
	}
	
	/**
	 * Auto update and state control method.
	 */
	public void auto() {
		if(action == ACTION_GEAR) {
			gearAction();
		} else if(action == ACTION_BOILER) {
			shootAction();
		}
		dt.update();
	}
	
	private void gearAction(){
		switch(state) {
			case DRIVE_TO_GEAR:
				Robot.getThrower().deployFlipper();
				Common.debug("Driving to the target :" + distance);
				dt.driveDistance(distance);
				state = TURN_TO_GEAR;
				break;
				
			case TURN_TO_GEAR:
				dt.drivebyPID();
				if (dt.driveComplete()) {
					Common.debug("AUTO:turningToTarget :"+turn);
					dt.turnTo(turn);
					state = ALIGN;
				}
				break;
				
			case ALIGN:
				dt.drivebyPID();
				if (dt.driveComplete()) {
					Common.debug("AUTO:GearTrackingStarted");
					GearVision.i.start();
					state = GEAR_VISION;
				}
				break;
			case GEAR_VISION:
				GearVision.i.track();
				dt.setDrive(GearVision.i.forward(),GearVision.i.turn() , GearVision.i.slide());
				break;
		}
	}
	private void shootAction(){
		switch(state) {
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
				}else{
					dt.drivebyPID();
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
				}else{
					dt.drivebyPID();
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
