package org.usfirst.frc.team4564.robot;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Common {
	private static final DateFormat formatter = new SimpleDateFormat("HH:mm:ss:SSS");

	public static void debug(String text) {
		System.out.println(formatter.format(new Date(time())) + text);
	}
	
	public static void dashStr(String title, String a) {
		SmartDashboard.putString(title + ": ", a);
	}
	
	public static void dashNum(String title, int a) {
		SmartDashboard.putNumber(title + ": ", a);
	}
	public static void dashNum(String title, double a) {
		SmartDashboard.putNumber(title + ": ", a);
	}
	public static void dashBool(String Title, boolean a) {
		SmartDashboard.putBoolean(Title, a);
	}
	public static long time() {
		return Calendar.getInstance().getTimeInMillis();
	}
}