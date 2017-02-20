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
	private static final int DRIVE_TO_GEAR = 0, TURN_TO_GEAR = 1, ALIGN = 2,GEAR_VISION = 3, 
			DRIVE_HOPPER = 4, SLIDE_HOPPER = 5,SLIDE_STOP=6, DRIVE_BOILER = 7, PIVOT_BOILER = 8, SHOOT = 9;
	private static final int LEFT = 0, CENTER = 1, RIGHT = 2;
	private static final int ACTION_GEAR = 0, ACTION_BOILER = 1;
	private static final int RED = 0, BLUE = 1;
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
		state = DRIVE_HOPPER;
		dt.getHeading().setHeadingHold(true);
//		startingPosition = (int)autoTable.getNumber("startingPosition", -1);
//		alliance = (int)autoTable.getNumber("alliance", -1);
//		action = (int)autoTable.getNumber("action", -1);
		startingPosition = LEFT;
		alliance = BLUE;
		action = ACTION_BOILER;
		
		Common.debug("AUTO:CALC");
		
		switch(startingPosition) {
		
			case RIGHT:
				if(alliance == RED) {
					distance = 89.5 - halfRobotWidth;
					turn = -60;
				} else {
					distance = 93.8 - halfRobotWidth;
					turn = -60;
				}
				break;
				
			case LEFT:
				if(alliance == RED) {
					distance = 93.8 - halfRobotWidth-4;
					turn = 60;
				} else {
					distance = 89.5 - halfRobotWidth;
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
			Robot.getThrower().state.update();
			Robot.getThrower().update();
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
				dt.driveDistance(-105 + 12 + halfRobotWidth);
				Robot.getThrower().retractFlipper();
				state = SLIDE_HOPPER;
				break;
			case SLIDE_HOPPER:
				if (dt.driveComplete()) {
					timer = Common.time() + 1500;
					state = SLIDE_STOP;
				}else{
					dt.drivebyPID();
				}
				break;
			case SLIDE_STOP:
				double slide = (alliance == RED) ? 1.0 : -1.0;
				dt.setDrive(0, -dt.getHeading().turnRate(), slide);
				if (Common.time() >= timer) {
					timer = Common.time()+1500;
					state = DRIVE_BOILER;
				}
				break;
			case DRIVE_BOILER:
				dt.setDrive(0, -dt.getHeading().turnRate(), ((alliance==RED) ? -0.5:0.5));
				if(Common.time() >= timer) {
					dt.driveDistance(60+5);
					state = PIVOT_BOILER;
				}
				break;
			
			case PIVOT_BOILER:
				if (dt.driveComplete()) {
					if(alliance == RED)
						dt.turnTo(45);
					else
						dt.turnTo(-45);
					state = SHOOT;
					timer = Common.time() + 2000;
				}else{
					dt.drivebyPID();
				}
				break;
			case SHOOT:
				Common.debug("AUTO:CASE SHOOT");
				if (timer <= Common.time()) {
					Common.debug("AUTO: fireing");
					dt.setDrive(-0.70,-dt.getHeading().turnRate(),0);
					Robot.getThrower().state.fire();
				}else{
					Common.debug("AUTO: drivingByPID");
					dt.drivebyPID();
				}
				break;
		}
	}
}
