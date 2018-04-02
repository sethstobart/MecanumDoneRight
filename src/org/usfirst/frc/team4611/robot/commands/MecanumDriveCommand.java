package org.usfirst.frc.team4611.robot.commands;

import org.usfirst.frc.team4611.robot.OI;
import org.usfirst.frc.team4611.robot.subsystems.OzoneMecanumDriveTrainTalon;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class MecanumDriveCommand extends Command {
	
	Subsystem driveTrain = null;
	
	public MecanumDriveCommand(Subsystem _ozoneMecanumDriveTrain){
		driveTrain	= _ozoneMecanumDriveTrain;
		this.requires(driveTrain);
	}
	
	protected void execute() { 
		double y = OI.filter(OI.rightJoy.getX());
		double x = OI.filter(OI.leftJoy.getX());
		double z = OI.filter(OI.leftJoy.getY());
	    ((OzoneMecanumDriveTrainTalon) driveTrain).move(y, x, z);
	    //((OzoneMecanumDriveTrainVictor driveTrain))
	  }
	
	@Override
	protected boolean isFinished() {
		return false; //Don't stop running this command 
	}

}
