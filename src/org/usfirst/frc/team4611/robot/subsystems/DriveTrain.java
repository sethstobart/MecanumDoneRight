package org.usfirst.frc.team4611.robot.subsystems;

import org.usfirst.frc.team4611.robot.commands.MecanumDrive;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4611.robot.RobotMap;

public class DriveTrain extends Subsystem {

	public void move(double y, double x, double z) { //Grabs the left and right values that get passed by "TankDrive"
		 RobotMap.driveTrain.driveCartesian(-x * .75, z * .75, y * .75); //Use those values for the method "tankDrive" which calls for joystick values
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new MecanumDrive()); //This subsystem will automatically run this command 
	}
}
