package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;

public class Climber {
	private Spark climbMotor;
	private PowerDistributionPanel pdp;
	
	public Climber(){
		climbMotor = new Spark(Constants.PWM_CLIMB);
		pdp = new PowerDistributionPanel(Constants.CANID_PDP);
	}
	public void setPower(double power){
		climbMotor.set(power);
	}
	public double getPower(){
		return climbMotor.get();
	}
	
	public double getAmpage()
	{
		return pdp.getCurrent(Constants.CLIMBER_POW_CHANNEL);
	}

}
