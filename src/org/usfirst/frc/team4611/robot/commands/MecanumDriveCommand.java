package org.usfirst.frc.team4611.robot.commands;

import org.usfirst.frc.team4611.robot.OI;
import org.usfirst.frc.team4611.robot.Robot;
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
		double YVal = -(OI.leftJoy.getY()); 
		double XVal = (OI.leftJoy.getX());
		double ZVal = (OI.rightJoy.getX());
		double velocity1 = 4*(1200 * (YVal + XVal + ZVal));
		double velocity2 = 4*(1200 * (YVal - XVal - ZVal));
		double velocity3 = 4*(1200 * (YVal + XVal - ZVal));
		double velocity4 = 4*(1200 * (YVal - XVal + ZVal));
	    ((OzoneMecanumDriveTrainTalon) driveTrain).setRampRate(0);
	    ((OzoneMecanumDriveTrainTalon) driveTrain).velocityDrive(velocity1, velocity2, velocity3, velocity4);
	    //((OzoneMecanumDriveTrainVictor driveTrain))
	  }
	
	@Override
	protected boolean isFinished() {
		return false; //Don't stop running this command 
	}

}
