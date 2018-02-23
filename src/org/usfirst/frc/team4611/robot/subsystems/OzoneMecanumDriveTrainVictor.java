package org.usfirst.frc.team4611.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class OzoneMecanumDriveTrainVictor extends Subsystem implements IOzoneSubsystem{
	
	private Victor drivetrainFL_Victor;
	private Victor drivetrainFR_Victor;
	private Victor drivetrainBL_Victor;
	private Victor drivetrainBR_Victor;

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void wire(Map<String, Object> wireMap) throws MissingWiringInstructionException {
		
		/**
		 * get ports for stuff out of wireMap
		 * throw an exception if my needs are not in there
		 */
		Integer blPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "backLeftMotor");
		Integer brPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "backrightMotor");
		Integer flPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "frontLeftMotor");
		Integer frPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "frontLeftMotor");
		
		if (blPort == null || brPort == null || flPort == null || frPort == null) {
			throw new MissingWiringInstructionException("No port provided for " + this.getClass().getName() + "." + "backLeftMotor");
		}

		drivetrainFL_Victor = new Victor(flPort.intValue());
		drivetrainFR_Victor = new Victor(frPort.intValue());
		drivetrainBL_Victor = new Victor(blPort.intValue());
		drivetrainBR_Victor = new Victor(brPort.intValue());		
	}

}
