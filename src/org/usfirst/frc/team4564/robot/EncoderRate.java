package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderRate implements Runnable {
	//Time to wait between loops in nanoseconds
	private long delay;
	private boolean threadSafe = true;
	
	private Encoder encoder;
	//Time of previous read in nanoseconds
	private long previousTime;
	//Counts of previous read
	private int previousCount;
	//Rate of the encoder in rpm
	private double rate;
	
	public EncoderRate(Encoder encoder) {
		this.encoder = encoder;
		previousCount = encoder.get();
		previousTime = System.nanoTime();
	}

	public void run() {
		while(true) {
			int deltaCount = encoder.get() - previousCount;
			long deltaTime = System.nanoTime() - previousTime;
			previousCount = encoder.get();
			previousTime = System.nanoTime();
			
			threadSafe = false;
			
			rate = 1.0 * deltaCount / deltaTime * 1000000000 * 60;
			
			threadSafe = true;
			
			long time = System.nanoTime();
			while (System.nanoTime() < time + delay) {
				int i = 1 + 1;
			}
		}
	}
	
	public double getRate() {
		while(!threadSafe) {
			System.out.println("notSafe");
			int i = 1 + 1;
		}
		return this.rate;
	}
	
	public void setSampleRate(int sampleRate) {
		delay = 1000000000 / sampleRate;
	}
}
