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
	private static final int DRIVE_TO_GEAR = 0, TURN_TO_GEAR = 1, ALIGN = 2,GEAR_VISION = 3, BACKUP = 4,FIRST_TURN = 5,SLIDE_OFF = 6,DRIVE_FORWARD =7,STOP = 8,
			DRIVE_HOPPER = 9, SLIDE_HOPPER = 10,SLIDE_STOP = 11,SLIDE_AWAY=12, DRIVE_BOILER = 13, PIVOT_BOILER = 14,SPIN_UP =15,SMASH = 16, SHOOT = 17;
	private static final int LEFT = 1, CENTER = 2, RIGHT = 3;
	private static final int ACTION_GEAR = 0, ACTION_BOILER = 1;
	private static final int RED = 0, BLUE = 1;
	private static final int halfRobotWidth = 36/2;
	
	private static final double WALL_TO_RIGHT_GEAR_RED = 88;
	private static final double WALL_TO_RIGHT_GEAR_BLUE = 84;
	private static final double WALL_TO_LEFT_GEAR_RED = 84;
	private static final double WALL_TO_LEFT_GEAR_BLUE = 88;
	private static final double WALL_TO_BASELINE = 114.3;
	private static final double WALL_TO_LAUNCHPAD_LINE = 184.8;
	private static final double FIELD_LENGTH = 652;
	private static final double FIELD_WIDTH = 324;
	
	private static final int POST_LEFT = -1, POST_STOP = 0 ,POST_RIGHT = 1;
	
	private static DriveTrain dt;
	private static NetworkTable autoTable;

	private int startingPosition; //Starting position 0-2, left to right from drive station.
	private int alliance; //0 for red, 1 for blue
	private int action; //0 for gears, 1 for shooting
	private int state; //Current auto state of the robot.
	private long timer;
	
	private double distance;
	private double turn;
	private boolean wasComplete;
	private int sprint;
	
	public Auto(DriveTrain drivetrain) {
		dt = drivetrain;
		autoTable = NetworkTable.getTable("auto");
	}
	
	public void init() {
		wasComplete = false;
		dt.getHeading().setHeadingHold(true);
		startingPosition = (int)autoTable.getNumber("startingPosition", -1);
		alliance = (int)autoTable.getNumber("alliance", -1);
		action = (int)autoTable.getNumber("action", -1);
		sprint = (int)autoTable.getNumber("postAction", POST_STOP);
//		startingPosition = CENTER;
//		alliance = RED;
//		action = ACTION_GEAR;
		
		Common.debug("AUTO:CALC");
		if (action == ACTION_GEAR) {
			state = DRIVE_TO_GEAR;
		}
		else {
			state = DRIVE_HOPPER;
		}
		
		switch(startingPosition) {
		
			case RIGHT:
				if(alliance == RED) {
					distance = WALL_TO_RIGHT_GEAR_RED - halfRobotWidth;
					turn = -60;
				} else {
					distance = WALL_TO_RIGHT_GEAR_BLUE - halfRobotWidth;
					turn = -60;
				}
				break;
				
			case LEFT:
				if(alliance == RED) {
					distance = WALL_TO_LEFT_GEAR_RED - halfRobotWidth;
					turn = 60;
				} else {
					distance = WALL_TO_LEFT_GEAR_BLUE - halfRobotWidth;
					turn = 60;
				}
				break;
				
			case CENTER:
				distance = 30;
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
			Robot.getThrower().retractFlipper();
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
					dt.relTurn(turn);
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
				if(GearVision.i.complete && !wasComplete){
					 timer = Common.time()+4000;
					 wasComplete = true;
					 Common.debug("AUTO:gearvision Complete");
				}
				/*if(Common.time() >= timer && wasComplete == true && ((sprint != POST_STOP)|| startingPosition != CENTER)){
					Common.debug("AUTO:backUp");
					state = BACKUP;
				}*/
				if(Common.time() >= timer && wasComplete == true){
					if (sprint != POST_STOP || startingPosition != CENTER) {
						Common.debug("AUTO:backUp");
						state = BACKUP;
					}
					else {
						state = STOP;
					}
				}
				else{
					GearVision.i.track();
					dt.setDrive(GearVision.i.forward(),GearVision.i.turn() , GearVision.i.slide());
				}
				break;
			
			case BACKUP:
				dt.driveDistance(-30);
				state = FIRST_TURN;
				break;
			case FIRST_TURN:
				if (dt.driveComplete()) {
					Common.debug("AUTO:firstTurn");
					if(startingPosition == CENTER){
						timer = Common.time() + 1500;
					}else {
						dt.relTurn(-turn);
					}
					Common.debug("AUTO:slide_off");
					state = SLIDE_OFF;
				}else { 
					dt.drivebyPID();
				}
				break;
			case SLIDE_OFF:
				if(startingPosition == CENTER) {
					if(timer <= Common.time()){
						state = DRIVE_FORWARD;
						Common.debug("AUTO:DrivingForward");
					}else{
						dt.setDrive(0, -dt.getHeading().turnRate(), sprint);
					}
				}else{
					state = DRIVE_FORWARD;
					Common.debug("AUTO:DrivingForward");
				}
				break;
				
			case DRIVE_FORWARD:
				if (startingPosition == CENTER){
					dt.driveDistance(240);
					state = STOP;
				}else
				if(dt.driveComplete()){
					dt.driveDistance(240);
					state = STOP;
				}else{
					dt.drivebyPID();
				}
				break;
			case STOP:
				dt.drivebyPID();
				break;
		}
	}
	private void shootAction(){
		switch(state) {
			case DRIVE_HOPPER:
				dt.driveDistance(-108+6 + halfRobotWidth);
				Robot.getThrower().retractFlipper();
				state = SLIDE_HOPPER;
				break;
			case SLIDE_HOPPER:
				if (dt.driveComplete()) {
					timer = Common.time() + 1000;
					state = SLIDE_STOP;
				}else{
					dt.drivebyPID();
				}
				break;
			case SLIDE_STOP:
				double slide = (alliance == RED) ? -1.0 : 1.0;
				dt.setDrive(0, -dt.getHeading().turnRate(), slide);
				if (Common.time() >= timer) {
					timer = Common.time()+2000;
					state = SLIDE_AWAY;
				}
				break;
			case SLIDE_AWAY:
				dt.setDrive(0, -dt.getHeading().turnRate(), 0);
				if (Common.time() >= timer) {
					timer = Common.time()+1500;
					state = DRIVE_BOILER;
				}
				break;
			case DRIVE_BOILER:
				dt.setDrive(0, -dt.getHeading().turnRate(), ((alliance==RED) ? 0.5:-0.5));
				if(Common.time() >= timer) {
					Robot.getThrower().intakeOn();
					dt.driveDistance(68-12-3);
					Robot.getThrower().state.spinUp();

					state = PIVOT_BOILER;
				}
				break;
			
			case PIVOT_BOILER:
				if (dt.driveComplete()) {
					if(alliance == RED)
						dt.turnTo(-45);
					else
						dt.turnTo(45);
					state = SPIN_UP;
				}else{
					dt.drivebyPID();
				}
				break;
			case SPIN_UP:
				Common.debug("AUTO:CASE SHOOT");
				if (dt.driveComplete() ) {
					Common.debug("AUTO: fireing");
					state = SHOOT;
				}else{
					Common.debug("AUTO: drivingByPID");
					dt.drivebyPID();
				}
				break;
			case SMASH:
				timer = Common.time()+0;
				state = SHOOT;
				break;
			case SHOOT:
				dt.setDrive(-0.70,-dt.getHeading().turnRate(),0);

				if(Common.time() >= timer){
					Robot.getThrower().state.fire();
				}
				break;
				
		}
	}
}
