package org.usfirst.frc.team4611.robot.subsystems;

import java.util.Map;

import org.usfirst.frc.team4611.robot.commands.MecanumDriveCommand;
import org.usfirst.frc.team4611.robot.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

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
public class OzoneMecanumDriveTrainTalon extends Subsystem implements IOzoneSubsystem {
	
	@Override
	public void wire(Map<String, Object> wireMap) throws MissingWiringInstructionException {
		
		/**
		 * get ports for stuff out of wireMap
		 * throw an exception if my needs are not in there
		 */
		Integer blPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "backLeftMotor"); //does return 10
		Integer brPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "backRightMotor");
		Integer flPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "frontLeftMotor");
		Integer frPort	= (Integer) wireMap.get(this.getClass().getName() + "." + "frontRightMotor");
		
		if (blPort == null || brPort == null || flPort == null || frPort == null) {
			throw new MissingWiringInstructionException("No port provided for " + this.getClass().getName());
		}
		
		driveTrainFL_Talon = new WPI_TalonSRX(flPort.intValue());
		driveTrainFR_Talon = new WPI_TalonSRX(frPort.intValue());
		driveTrainBL_Talon = new WPI_TalonSRX(blPort.intValue());
		driveTrainBR_Talon = new WPI_TalonSRX(brPort.intValue());
		
		driveTrainFL_Talon.setInverted(false);
		driveTrainBL_Talon.setInverted(false);
		driveTrainBR_Talon.setInverted(true);
		driveTrainFR_Talon.setInverted(true);
		
		driveTrainFL_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		driveTrainFR_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		driveTrainBL_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		driveTrainBR_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		config_kP(.65);
		config_kI(0);
		config_kD(0);
		
		driveTrainFL_Talon.setSensorPhase(true);
		driveTrainFR_Talon.setSensorPhase(true);
		driveTrainBL_Talon.setSensorPhase(true);
		driveTrainBR_Talon.setSensorPhase(true);
		
		mecanumDriveTrain = new MecanumDrive(driveTrainFL_Talon, driveTrainBL_Talon, driveTrainFR_Talon, driveTrainBR_Talon);	
	}

  	// motors
  	private  WPI_TalonSRX driveTrainFL_Talon;
	private  WPI_TalonSRX driveTrainFR_Talon;
	private  WPI_TalonSRX driveTrainBL_Talon;
	private  WPI_TalonSRX driveTrainBR_Talon;
	private MecanumDrive mecanumDriveTrain;


		public void move(double y, double x, double z) { 
			 mecanumDriveTrain.driveCartesian(y, x, z); 
		}
		
		public void moveGyro (double y, double x, double z, double gyroAngle) {
			mecanumDriveTrain.driveCartesian(y, x, z, gyroAngle);
		}
		
		public void movePolar (double mag, double angle, double z) {
			mecanumDriveTrain.drivePolar(mag, angle, z);
		}
		
		public void motionMagicStraight(double positionUnits) {
			this.driveTrainBL_Talon.set(ControlMode.MotionMagic, positionUnits);
			this.driveTrainBR_Talon.set(ControlMode.MotionMagic, -positionUnits);
			this.driveTrainFL_Talon.set(ControlMode.MotionMagic, positionUnits);
			this.driveTrainFR_Talon.set(ControlMode.MotionMagic, -positionUnits);
		}
		
		public void motionMagicStrafe(double positionUnits) {
			this.driveTrainFR_Talon.set(ControlMode.MotionMagic, positionUnits);
			this.driveTrainBR_Talon.set(ControlMode.MotionMagic, -positionUnits);
			this.driveTrainFL_Talon.set(ControlMode.MotionMagic, positionUnits);
			this.driveTrainBL_Talon.set(ControlMode.MotionMagic, -positionUnits);
		}
		
		public void resetEncoders() {
			this.driveTrainBL_Talon.setSelectedSensorPosition(0, 0, 0);
			this.driveTrainBR_Talon.setSelectedSensorPosition(0, 0, 0);
			this.driveTrainFL_Talon.setSelectedSensorPosition(0, 0, 0);
			this.driveTrainFR_Talon.setSelectedSensorPosition(0, 0, 0);
		}
		//reset to default 
		public void velocityDrive(double velocity1, double velocity2, double velocity3, double velocity4) {
			this.driveTrainBL_Talon.set(ControlMode.Velocity, velocity4);
			this.driveTrainBR_Talon.set(ControlMode.Velocity, velocity3);
			this.driveTrainFL_Talon.set(ControlMode.Velocity, velocity1);
			this.driveTrainFR_Talon.set(ControlMode.Velocity, velocity2);		
		}
		//reset to 0 before every motion magic command
		public void resetRampRate() {
			this.driveTrainBL_Talon.configClosedloopRamp(0, 0);
			this.driveTrainBR_Talon.configClosedloopRamp(0, 0);
			this.driveTrainFL_Talon.configClosedloopRamp(0, 0);
			this.driveTrainFR_Talon.configClosedloopRamp(0, 0);
		}
		//set back to default Ramp Rate
		public void setRampRate(double rate) {
			this.driveTrainBL_Talon.configClosedloopRamp(rate, 0);
			this.driveTrainBR_Talon.configClosedloopRamp(rate, 0);
			this.driveTrainFL_Talon.configClosedloopRamp(rate, 0);
			this.driveTrainFR_Talon.configClosedloopRamp(rate, 0);
		}
		
		public void rotate(double velocity) {
			this.driveTrainBL_Talon.set(ControlMode.Velocity, velocity);
			this.driveTrainBR_Talon.set(ControlMode.Velocity, velocity);
			this.driveTrainFL_Talon.set(ControlMode.Velocity, velocity);
			this.driveTrainFR_Talon.set(ControlMode.Velocity, velocity);

		}
		
		public void config_kP(double p) {
			this.driveTrainBL_Talon.config_kP(0, p, 0);
			this.driveTrainBR_Talon.config_kP(0, p, 0);
			this.driveTrainFL_Talon.config_kP(0, p, 0);
			this.driveTrainFR_Talon.config_kP(0, p, 0);
		}
		
		public void config_kI(double i) {
			this.driveTrainBL_Talon.config_kI(0, i, 0);
			this.driveTrainBR_Talon.config_kI(0, i, 0);
			this.driveTrainFL_Talon.config_kI(0, i, 0);
			this.driveTrainFR_Talon.config_kI(0, i, 0);
		}
		
		public void config_kD(double d) {
			this.driveTrainBL_Talon.config_kD(0, d, 0);
			this.driveTrainBR_Talon.config_kD(0, d, 0);
			this.driveTrainFL_Talon.config_kD(0, d, 0);
			this.driveTrainFR_Talon.config_kD(0, d, 0);
		}
		
		public void logSpeed() {
		  	double blSpeed, brSpeed, flSpeed, frSpeed;
	    	
	    	blSpeed	= this.driveTrainBL_Talon.getMotorOutputPercent();
	    	brSpeed	= this.driveTrainBR_Talon.getMotorOutputPercent();
	    	flSpeed	= this.driveTrainFL_Talon.getMotorOutputPercent();
	    	frSpeed	= this.driveTrainFR_Talon.getMotorOutputPercent();

	    	Logger.log("motorSpeeds [bl, br, fl, fr] ["
														+ blSpeed + ", "
														+ brSpeed + ", "
														+ flSpeed + ", "
														+ frSpeed + ']', "DriveTrain");
		}
		
		public void logPosition() {
	    	double blPosition, brPosition, flPosition, frPosition;
	    	
	    	blPosition	= this.driveTrainBL_Talon.getSelectedSensorPosition(0);
	    	brPosition	= this.driveTrainBR_Talon.getSelectedSensorPosition(0);
	    	flPosition	= this.driveTrainFL_Talon.getSelectedSensorPosition(0);
	    	frPosition	= this.driveTrainFR_Talon.getSelectedSensorPosition(0);
	      	Logger.log("motorPositions [bl, br, fl, fr] ["
														+ blPosition + ", "
														+ brPosition + ", "
														+ flPosition + ", "
														+ frPosition + ']', "DriveTrain");
		}
		
		public double getAveragePosition() {
			double encoderPositionAverage = (Math.abs(this.driveTrainBL_Talon.getSelectedSensorPosition(0)) +
			    	Math.abs(this.driveTrainBR_Talon.getSelectedSensorPosition(0)) +
			       	Math.abs(this.driveTrainFL_Talon.getSelectedSensorPosition(0)) +
			       	Math.abs(this.driveTrainFR_Talon.getSelectedSensorPosition(0))) / 4;
			return encoderPositionAverage;
		}
		
		public double getAverageSpeed() {
			double encoderSpeedAverage = (Math.abs(this.driveTrainBL_Talon.getSelectedSensorVelocity(0)) +
			    	Math.abs(this.driveTrainBR_Talon.getSelectedSensorVelocity(0)) +
			       	Math.abs(this.driveTrainFL_Talon.getSelectedSensorVelocity(0)) +
			       	Math.abs(this.driveTrainFR_Talon.getSelectedSensorVelocity(0))) / 4;
			return encoderSpeedAverage;
		}
		
		/**
		   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
		   * between the deadband and 1.0 is scaled from 0.0 to 1.0. 
		   *
		   * @param value    value to clip
		   * @param deadband range around zero
		   */
		protected double applyDeadband(double value, double deadband) {
		    if (Math.abs(value) > deadband) {
		      if (value > 0.0) {
		        return (value - deadband) / (1.0 - deadband);
		      } else {
		        return (value + deadband) / (1.0 - deadband);
		      }
		    } else {
		      return 0.0;
		    }
		}
		
		public boolean isTargetSpeedWithinThreshold(double speed){
			if(Math.abs(speed) > 200) {
				return true;
			}
			
			else {
				return false;
			}
		}
		
		@Override
		protected void initDefaultCommand() {
			setDefaultCommand(new MecanumDriveCommand(this));
		}
	}