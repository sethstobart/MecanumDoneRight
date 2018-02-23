package org.usfirst.frc.team4611.robot.subsystems;

public class SubsystemFactory {

		
		private boolean useTalon1 = true;
		

		public SubsystemFactory() {
		}
		
		public IOzoneSubsystem createDriveTrain() {
			
			// if we have Talons
			if (useTalon1)
				return new OzoneMecanumDriveTrainTalon();
			else {
				return new OzoneMecanumDriveTrainTalon2();
		
		}
	}

}
