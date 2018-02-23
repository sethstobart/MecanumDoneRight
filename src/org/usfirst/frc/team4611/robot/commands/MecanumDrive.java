package org.usfirst.frc.team4611.robot.commands;

import org.usfirst.frc.team4611.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MecanumDrive extends Command{
	
	public MecanumDrive(){
		this.requires(Robot.tankDrive); //This command uses this subsystem
	}
	
	protected void execute() { //execute is called every 20 miliseconds
		double y = Robot.oi.filter(Robot.oi.rightJoy.getX()); //Grab the Y value of the joystick and pass 
		double x = Robot.oi.filter(Robot.oi.leftJoy.getX());
		double z = Robot.oi.filter(Robot.oi.leftJoy.getY());; //it through the filter 
	    Robot.tankDrive.move(y, x , z); //Then pass that double to the method "move" in tankDrive
	  }
	
	@Override
	protected boolean isFinished() {
		return false; //Don't stop running this command 
	}

}
