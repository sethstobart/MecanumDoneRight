package org.usfirst.frc.team4611.robot.subsystems;


import java.util.Map;

import org.usfirst.frc.team4611.robot.commands.MecanumDriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A class for driving Mecanum drive platforms.
 *
 * <p>Mecanum drives are rectangular with one wheel on each corner. Each wheel has rollers toed in
 * 45 degrees toward the front or back. When looking at the wheels from the top, the roller axles
 * should form an X across the  Each drive() function provides different inverse kinematic
 * relations for a Mecanum drive 
 *
 * <p>Drive base diagram:
 * <pre>
 * \\_______/
 * \\ |   | /
 *   |   |
 * /_|___|_\\
 * /       \\
 * </pre>
 *
 * <p>Each drive() function provides different inverse kinematic relations for a Mecanum drive
 *  Motor outputs for the right side are negated, so motor direction inversion by the user is
 * usually unnecessary.
 *
 * <p>This library uses the NED axes convention (North-East-Down as external reference in the world
 * frame): http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * <p>The positive X axis points ahead, the positive Y axis points right, and the positive Z axis
 * points down. Rotations follow the right-hand rule, so clockwise rotation around the Z axis is
 * positive.
 *
 * <p>Inputs smaller then {@value edu.wpi.first.wpilibj.drive.RobotDriveBase#kDefaultDeadband} will
 * be set to 0, and larger values will be scaled so that the full range is still used. This
 * deadband value can be changed with {@link #setDeadband}.
 *
 * <p>RobotDrive porting guide:
 * <br>In MecanumDrive, the right side speed controllers are automatically inverted, while in
 * RobotDrive, no speed controllers are automatically inverted.
 * <br>{@link #driveCartesian(double, double, double, double)} is equivalent to
 * {@link edu.wpi.first.wpilibj.RobotDrive#mecanumDrive_Cartesian(double, double, double, double)}
 * if a deadband of 0 is used, and the ySpeed and gyroAngle values are inverted compared to
 * RobotDrive (eg driveCartesian(xSpeed, -ySpeed, zRotation, -gyroAngle).
 * <br>{@link #drivePolar(double, double, double)} is equivalent to
 * {@link edu.wpi.first.wpilibj.RobotDrive#mecanumDrive_Polar(double, double, double)} if a
 * deadband of 0 is used.
 */
public class OzoneMecanumDriveTrainTalon2 extends Subsystem implements IOzoneSubsystem{

  	// motors
  	private  WPI_TalonSRX driveTrainFL_Talon;
	private  WPI_TalonSRX driveTrainFR_Talon;
	private  WPI_TalonSRX driveTrainBL_Talon;
	private  WPI_TalonSRX driveTrainBR_Talon;
	
	int velocityInvert1 = 1;
	int velocityInvert2 = -1;
	int velocityInvert3 = -1;
	int velocityInvert4 = 1;
	
	private boolean m_reported = false;
  
  /**
   * Construct a MecanumDrive.
   *
   * <p>If a motor needs to be inverted, do so before passing it in.
   */
  public OzoneMecanumDriveTrainTalon2 (){

	/* **
	 * this was refactored from WPI where Mecanum extended Sendable. the purpose was to get the motors in smart dashboard.
    addChild(m_frontLeftMotor);
    addChild(m_rearLeftMotor);
    addChild(m_frontRightMotor);
    addChild(m_rearRightMotor);
    */

  }
  


  /* (non-Javadoc)
 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#driveCartesian(double, double, double, double)
 */
  public void driveCartesian(double YVal, double XVal, double ZVal, double gyroAngle) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 4,
                 tInstances.kRobotDrive_MecanumCartesian);
      m_reported = true;
    }
    
	double velocity1;
	double velocity2;
	double velocity3;
	double velocity4;
	double YValScaler1 = 1;
	double XValScaler1 = 1;
	double YValScaler2 = 1;
	double XValScaler2 = 1;
	double ZValScaler = 1;
	double	maxRPM	= 750;

	
	// Blaine and Halter's magic math
	velocity1 = 4*(maxRPM * YVal * YValScaler1 + XVal * XValScaler1 + ZVal * ZValScaler) * (velocityInvert1);
	velocity2 = 4*(maxRPM * YVal * YValScaler2 - XVal * XValScaler2 - ZVal * ZValScaler) * (velocityInvert2); 
	velocity3 = 4*(maxRPM * YVal * YValScaler2 + XVal * XValScaler2 - ZVal * ZValScaler) * (velocityInvert3);
	velocity4 = 4*(maxRPM * YVal * YValScaler1 - XVal * XValScaler1 + ZVal * ZValScaler) * (velocityInvert4);

	setRampRate();
	velocityDrive(velocity1, velocity2, velocity3, velocity4);    
  }

  
  /* (non-Javadoc)
 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#drivePolar(double, double, double)
 */
  public void drivePolar(double magnitude, double angle, double zRotation) {
    if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive, 4, tInstances.kRobotDrive_MecanumPolar);
      m_reported = true;
    }

    driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                   magnitude * Math.cos(angle * (Math.PI / 180.0)), zRotation, 0.0);
  }

  /* (non-Javadoc)
 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#stopMotor()
 */
public void stopMotor() {
    driveTrainFL_Talon.stopMotor();
    driveTrainFR_Talon.stopMotor();
    driveTrainBL_Talon.stopMotor();
    driveTrainBR_Talon.stopMotor();
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MecanumDrive");
    builder.addDoubleProperty("Front Left Motor Speed", driveTrainFL_Talon::get,
        driveTrainFL_Talon::set);
    builder.addDoubleProperty("Front Right Motor Speed", () -> -driveTrainFR_Talon.get(),
        value -> driveTrainFR_Talon.set(-value));
    builder.addDoubleProperty("Rear Left Motor Speed", driveTrainBL_Talon::get,
        driveTrainBL_Talon::set);
    builder.addDoubleProperty("Rear Right Motor Speed", () -> -driveTrainBR_Talon.get(),
        value -> driveTrainBR_Talon.set(-value));
  }
  
  /* (non-Javadoc)
 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#move(double, double, double)
 */
public void move(double y, double x, double z) { //Grabs the left and right values that get passed by "TankDrive"
		 driveCartesian(y, x, z, 0.0); //Use those values for the method "tankDrive" which calls for joystick values
	}
	
	/* (non-Javadoc)
	 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#moveGyro(double, double, double, double)
	 */
	public void moveGyro (double y, double x, double z, double gyroAngle) {
		driveCartesian(y, x, z, gyroAngle);
	}
	
	/* (non-Javadoc)
	 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#movePolar(double, double, double)
	 */
	public void movePolar (double mag, double angle, double z) {
		drivePolar(mag, angle, z);
	}
	
	/* (non-Javadoc)
	 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#motionMagicStraight(int)
	 */
	public void motionMagicStraight(int positionUnits) {
		driveTrainBL_Talon.set(ControlMode.MotionMagic, positionUnits);
		driveTrainBR_Talon.set(ControlMode.MotionMagic, -positionUnits);
		driveTrainFL_Talon.set(ControlMode.MotionMagic, positionUnits);
		driveTrainFR_Talon.set(ControlMode.MotionMagic, -positionUnits);
	}
	
	/* (non-Javadoc)
	 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#motionMagicStrafe(int)
	 */
	public void motionMagicStrafe(int positionUnits) {
		driveTrainBL_Talon.set(ControlMode.MotionMagic, -positionUnits);
		driveTrainBR_Talon.set(ControlMode.MotionMagic, -positionUnits);
		driveTrainFL_Talon.set(ControlMode.MotionMagic, positionUnits);
		driveTrainFR_Talon.set(ControlMode.MotionMagic, positionUnits);
	}
	
	/* (non-Javadoc)
	 * @see org.usfirst.frc.team4611.subsystems.IOzoneMecanumDrive#velocityDrive(double, double, double, double)
	 */
	public void velocityDrive(double velocity1, double velocity2, double velocity3, double velocity4) {
		driveTrainBL_Talon.set(ControlMode.Velocity, velocity4);
		driveTrainBR_Talon.set(ControlMode.Velocity, velocity3);
		driveTrainFL_Talon.set(ControlMode.Velocity, velocity1);
		driveTrainFR_Talon.set(ControlMode.Velocity, velocity2);
		
		String out = "";
		

		out += driveTrainBL_Talon.getSelectedSensorVelocity(0) + "," + velocity4+","+driveTrainBL_Talon.getMotorOutputPercent() + ",";
		out += driveTrainBR_Talon.getSelectedSensorVelocity(0) + "," + velocity3+","+driveTrainBR_Talon.getMotorOutputPercent() + ",";
		out += driveTrainFL_Talon.getSelectedSensorVelocity(0) + "," + velocity1+","+driveTrainFL_Talon.getMotorOutputPercent() + ",";
		out += driveTrainFR_Talon.getSelectedSensorVelocity(0) + "," + velocity2+","+driveTrainFR_Talon.getMotorOutputPercent() + ",";
		
		System.out.println(out);
		
	}
	//reset to 0 before every motion magic command
		public void resetRampRate() {
			driveTrainBL_Talon.configClosedloopRamp(0, 0);
			driveTrainBR_Talon.configClosedloopRamp(0, 0);
			driveTrainFL_Talon.configClosedloopRamp(0, 0);
			driveTrainFR_Talon.configClosedloopRamp(0, 0);
		}
		//set back to default Ramp Rate
		public void setRampRate() {
			double rampSeconds = 0.5; //(double)getValue(mecanumSubTable, rampTime);
			driveTrainBL_Talon.configClosedloopRamp(rampSeconds, 0);
			driveTrainBR_Talon.configClosedloopRamp(rampSeconds, 0);
			driveTrainFL_Talon.configClosedloopRamp(rampSeconds, 0);
			driveTrainFR_Talon.configClosedloopRamp(rampSeconds, 0);
		}
		
		public void rotate(double velocity) {
			driveTrainBL_Talon.set(ControlMode.Velocity, velocity);
			driveTrainBR_Talon.set(ControlMode.Velocity, velocity);
			driveTrainFL_Talon.set(ControlMode.Velocity, velocity);
			driveTrainFR_Talon.set(ControlMode.Velocity, velocity);

		}
		
		public void logSpeed() {
		  	double blSpeed, brSpeed, flSpeed, frSpeed;
	    	
	    	blSpeed	= driveTrainBL_Talon.get();
	    	brSpeed	= driveTrainBR_Talon.get();
	    	flSpeed	= driveTrainFL_Talon.get();
	    	frSpeed	= driveTrainFR_Talon.get();

	    	System.out.println(this.getClass().getName() + "isFinished() : motorSpeeds [bl, br, fl, fr] ["
	    																			+ blSpeed + ", "
	    																			+ brSpeed + ", "
	    																			+ flSpeed + ", "
	    																			+ frSpeed + ']');
		}
		
		public void logPosition() {
	    	double blPosition, brPosition, flPosition, frPosition;
	    	
	    	blPosition	= driveTrainBL_Talon.getSelectedSensorPosition(0);
	    	brPosition	= driveTrainBR_Talon.getSelectedSensorPosition(0);
	    	flPosition	= driveTrainFL_Talon.getSelectedSensorPosition(0);
	    	frPosition	= driveTrainFR_Talon.getSelectedSensorPosition(0);
	      	System.out.println(this.getClass().getName() + "isFinished() : motorPositions [bl, br, fl, fr] ["
	      																			+ blPosition + ", "
	      																			+ brPosition + ", "
	      																			+ flPosition + ", "
	      																			+ frPosition + ']');
		}
		


	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new MecanumDriveCommand(this));
	}



	@Override
	public void wire(Map<String, Object> wireMap) throws MissingWiringInstructionException {
		
		/**
		 * get ports for stuff out of wireMap
		 * throw an exception if my needs are not in there
		 */
		Integer blPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "backLeftMotor");
		
		if (blPort == null) {
			throw new MissingWiringInstructionException("No port provided for " + this.getClass().getName() + "." + "backLeftMotor");
		}

		driveTrainFL_Talon = new WPI_TalonSRX(12);
		driveTrainFR_Talon = new WPI_TalonSRX(13);
		driveTrainBL_Talon = new WPI_TalonSRX(blPort.intValue());
		driveTrainBR_Talon = new WPI_TalonSRX(11);
		
	}
}
