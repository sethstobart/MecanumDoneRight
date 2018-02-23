package org.usfirst.frc.team4611.robot.commands;

import org.usfirst.frc.team4611.robot.OI;
import org.usfirst.frc.team4611.robot.subsystems.OzoneMecanumDriveTrainTalon;

import edu.wpi.first.wpilibj.command.Command;

public class MecanumDriveCommand extends Command {
	
	OzoneMecanumDriveTrainTalon	driveTrain	= null;
	
	public MecanumDriveCommand(OzoneMecanumDriveTrainTalon _ozoneMecanumDriveTrainTalon){
		driveTrain	= _ozoneMecanumDriveTrainTalon;
		this.requires(driveTrain);
	}
	
	protected void execute() { 
		double y = OI.filter(OI.rightJoy.getX());
		double x = OI.filter(OI.leftJoy.getX());
		double z = OI.filter(OI.leftJoy.getY());
	    driveTrain.move(y, x , z);
	  }
	
	@Override
	protected boolean isFinished() {
		return false; //Don't stop running this command 
	}

}
