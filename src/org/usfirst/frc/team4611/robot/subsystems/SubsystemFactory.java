package org.usfirst.frc.team4611.robot.subsystems;

public class SubsystemFactory {


		public SubsystemFactory() {
		}
		
		public IOzoneSubsystem createDriveTrain(String speedController) {
			
			// if we have Talons
			if (speedController.equals("talon"))
				return new OzoneMecanumDriveTrainTalon();
			if (speedController.equals("victor")) 
				return new OzoneMecanumDriveTrainVictor();
			else {
				return new OzoneMecanumDriveTrainVictor(); // I realize it repeats but I dont know what the third case should be
			}
	}

}
