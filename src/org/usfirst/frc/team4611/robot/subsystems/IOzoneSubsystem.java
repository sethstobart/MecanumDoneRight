package org.usfirst.frc.team4611.robot.subsystems;

import java.util.Map;

public interface IOzoneSubsystem {
	
	public void wire(Map<String, Object> wireMap) throws MissingWiringInstructionException;
}
