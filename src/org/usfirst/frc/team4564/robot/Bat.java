package org.usfirst.frc.team4564.robot;
import edu.wpi.first.wpilibj.AnalogInput;

public class Bat {
	// 10 cycles per second update rate
	int cycleDelay = (int)Constants.REFRESH_RATE/10;
	int currentCycle = 0;
	
	// Device setup
	AnalogInput sonicRight = new AnalogInput (Constants.SONIC_PIN);  //For defense detection
	//DigitalOutput sonicRightEnable = new DigitalOutput(Constants.DIO_LEFT_SONIC_ENABLE);
	
	//Constants
	private static final double CORRECTION = 1 / 1.04;
	private static final double VOLTS_PER_INCH = 5.0 / 1024 * 2.54 * CORRECTION; // Volts per inch constan
	
	// Distance right side of robot is from defensive wall
	public double getDistance() {
		return sonicRight.getVoltage() / VOLTS_PER_INCH;
	}

}