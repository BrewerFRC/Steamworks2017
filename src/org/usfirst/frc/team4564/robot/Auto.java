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
	private double slideOff;
	
	public Auto(DriveTrain drivetrain) {
		dt = drivetrain;
		autoTable = NetworkTable.getTable("auto");
	}
	
	public void init() {
		dt.getHeading().setHeadingHold(true);
//		startingPosition = (int)autoTable.getNumber("startingPosition", -1);
//		alliance = (int)autoTable.getNumber("alliance", -1);
//		action = (int)autoTable.getNumber("action", -1);
		startingPosition = CENTER;
		alliance = RED;
		action = ACTION_GEAR;
		slideOff = 1;
		
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
					distance = 89.5 - halfRobotWidth - 10;
					turn = -60;
				} else {
					distance = 93.8 - halfRobotWidth-8;
					turn = -60;
				}
				break;
				
			case LEFT:
				if(alliance == RED) {
					distance = 93.8 - halfRobotWidth-8;
					turn = 60;
				} else {
					distance = 89.5 - halfRobotWidth-10;
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
					timer = Common.time()+5000;
				}
				break;
			case GEAR_VISION:
				if(Common.time() >= timer && GearVision.i.complete){
					state = BACKUP;
				}else{
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
					if(startingPosition == CENTER){
						timer = Common.time() + 1500;
					}else {
						dt.relTurn(-turn);
					}
					state = SLIDE_OFF;
				}else { 
					dt.drivebyPID();
				}
				break;
			case SLIDE_OFF:
				if(startingPosition == CENTER) {
					if(timer <= Common.time()){
						state = DRIVE_FORWARD;
					}else{
						dt.setDrive(0, -dt.getHeading().turnRate(), slideOff);
					}
				}else{
					state = DRIVE_FORWARD;
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
				dt.driveDistance(-105 + 7 + halfRobotWidth);
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
				double slide = (alliance == RED) ? -1.0 : 1.0;
				dt.setDrive(0, -dt.getHeading().turnRate(), slide);
				if (Common.time() >= timer) {
					timer = Common.time()+1500;
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
					dt.driveDistance(60+10);
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
				timer = Common.time()+250;
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
