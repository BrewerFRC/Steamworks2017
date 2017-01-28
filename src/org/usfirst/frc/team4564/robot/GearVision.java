package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class GearVision {
	public static final int MIN_ALIGN_DISTANCE = 35;
	public static final int MAX_ALIGN_DISTANCE = 150;
	
	private NetworkTable table;
	private DriveTrain dt;
	
	private boolean reached;
	private boolean aligned;
	
	public GearVision() {
		table = NetworkTable.getTable("visionTracking");
		dt = Robot.getDriveTrain();
	}
	
	public boolean checkReady() {
		double distance = table.getNumber("distance", 0);
		return distance > MIN_ALIGN_DISTANCE && distance < MAX_ALIGN_DISTANCE;
	}
	
	public void start() {
		DriveTrain.getHeading().setHeadingHold(true);
	}
	
	public void reset() {
		DriveTrain.getHeading().setHeadingHold(false);
		reached = false;
		aligned = false;
	}

	public void track() {
		double turn = 0;
		double slide = table.getNumber("slideRate", 0) / 1.5;
		double distance = table.getNumber("distance", 0);
		if (distance == 704) {
			return;
		}
		int sign = (slide < 0) ? -1 : 1;
		if(!(slide == 0)) {
			slide = sign * (Math.abs(slide) + 0.30);
		}
		turn = table.getNumber("rateTurn", 0)/50.0;
		Common.dashNum("Vision Turn: ", turn);
		Common.dashNum("Turn Rate:", -DriveTrain.getHeading().turnRate());
		Common.dashNum("Target: ", DriveTrain.getHeading().getTargetAngle());
		Common.dashNum("Angle", DriveTrain.getHeading().getAngle());
		if (distance <= MIN_ALIGN_DISTANCE) {
			reached = true;
		}
		if (reached) {
			if((slide == 0 && (Math.abs(turn) <= 3)) || aligned){
				aligned = true;
				dt.setDrive(-.40 , -DriveTrain.getHeading().turnRate(), 0);
			} else {
				dt.setDrive(0, -DriveTrain.getHeading().turnRate(), -slide);
				//dt.setDrive(0, -DriveTrain.getHeading().turnRate(), 0);
			}
		}
		else {
			double fwdpow = (distance-28)*0.004+.43;
			if(fwdpow > .58) {
				fwdpow = .58;
			}
			dt.setDrive(-fwdpow, -DriveTrain.getHeading().turnRate(), -slide);
			//dt.setDrive(0, -DriveTrain.getHeading().turnRate(), 0);
			if(distance > 45) {
		
				//DriveTrain.getHeading().incrementTargetAngle(turn);
			}
		}
	}
}
