package org.usfirst.frc.team4611.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;


/**
 * This is where we create all of out buttons and joysticks and 
 * set up the "UI" of the bot for the drivers. You're gonna end up 
 * here a lot when people complain about buttons needing to be changed
 */

public class OI {
	public static Joystick leftJoy;
	public static Joystick rightJoy;
	//Joystick ports
	static int leftJoyPort = 0; //Joystick can be found on this port. The ports aren't physical plugs
	static int rightJoyPort = 1; //But rather decided from the drivers station by the drivers
	
	public OI (){
		leftJoy = new Joystick(leftJoyPort); //The left joystick exists on this port in robot map
		rightJoy = new Joystick(rightJoyPort); //The right joystick exists on this port in robot map
	}
	
	public static double filter(double raw) //We pass joystick values through the filter here and edit the raw value
    {
        if (Math.abs(raw) < .15) {
            return 0; //If the value passed is less than 15% ignore it. This is reffered to as a deadzone
        } else {
            return  raw * (1); //Set the output to a ceratin percent of of the input
        }
    }
	
	public static double strafeFilter(double raw) {
		if (Math.abs(raw) < .15) {
            return 0; //If the value passed is less than 15% ignore it. This is reffered to as a deadzone
        } else {
            return  raw * 2; //Set the output to a ceratin percent of of the input
        }
	}	
	
	public static double rotateFilter(double raw) {
		if (Math.abs(raw) < .15) {
            return 0; //If the value passed is less than 15% ignore it. This is reffered to as a deadzone
        } else {
            return  raw * 1; //Set the output to a ceratin percent of of the input
        }
}
}
