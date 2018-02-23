package org.usfirst.frc.team4611.robot;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The RobotMap is where we declare what our constants are
 * It's where we set which PWM the Victors are plugged into and what ports are which joysticks etc.
 * This is where you're probably gonna end up when incorrect motors are running or we change ports for certain motors.
 */
public class RobotMap {
	public static MecanumDrive driveTrain;
	
	public static Victor driveTrainFL;
	public static Victor driveTrainFR;
	public static Victor driveTrainBL;
	public static Victor driveTrainBR;
	
	//Joystick ports
	public static int leftJoyPort = 0; //Joystick can be found on this port. The ports aren't physical plugs
	public static int rightJoyPort = 1; //But rather decided from the drivers station by the drivers
	
	public static void init () {
		//PWM Ports
		//PWM ports are physically on the rio and the number on the port should match with the int in code
		driveTrainFL = new Victor(3);
		driveTrainFR = new Victor(0);
		driveTrainBL = new Victor(2);
		driveTrainBR = new Victor(1);
		
		//CAN Ports
		//CAN ports are decided via software in the roborio web interface 
		
		//Objects
		driveTrain =  new MecanumDrive(driveTrainFL, driveTrainBL, driveTrainFR, driveTrainBR);
		
		//Constants
				
	}
}
