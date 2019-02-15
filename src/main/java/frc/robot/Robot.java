/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The ArcadeDrive_AuxFeedForward example demonstrates the ability to create an Arcade
 * Drive based robot by using the Talon's and Victor's Feed Forward feature. 
 * 
 * The Auxiliary Feed Forward feature allows the user to apply more or less output on 
 * the motor controllers current control mode's output, regardless of it being open or closed. 
 * 
 * Controls:
 * Left Joystick Y-Axis: Drive robot forward and reverse
 * Right Joystick X-Axis: Turn robot right and left
 * 
 * Supported Version:
 * 	- Talon SRX: 4.0
 * 	- Victor SPX: 4.0
 * 	- Pigeon IMU: 4.0
 * 	- CANifier: 4.0
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
public class Robot extends TimedRobot {
	/** Hardware, either Talon could be a Victor */
	WPI_TalonSRX _leftMaster = new WPI_TalonSRX(11);
	WPI_TalonSRX _rightMaster = new WPI_TalonSRX(10);
	WPI_VictorSPX  _leftFollow = new WPI_VictorSPX (13);
	WPI_VictorSPX  _rightFollow = new WPI_VictorSPX (12);

	VictorSP climbFront = new VictorSP(0);
	VictorSP climbBack = new VictorSP(1);

	VictorSP climbDrive = new VictorSP(2);
	VictorSP elevatorDrive = new VictorSP(3);

	Joystick _game2 = new Joystick(1);
	Joystick _gamepad = new Joystick(0);
	DifferentialDrive _drive = new DifferentialDrive(_leftMaster, _rightMaster);

	Encoder sampleEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	

	@Override
	public void robotInit() {
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		//_leftMaster.set(ControlMode.PercentOutput, 0);
		//_rightMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		_leftFollow.configFactoryDefault();
		_rightFollow.configFactoryDefault();
		
		_leftFollow.follow(_leftMaster);
		_rightFollow.follow(_rightMaster);
		
		_leftMaster.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
		_rightMaster.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
		_leftFollow.setInverted(InvertType.FollowMaster);
		_rightFollow.setInverted(InvertType.FollowMaster);

		/* diff drive assumes (by default) that 
			right side must be negative to move forward.
			Change to 'false' so positive/green-LEDs moves robot forward  */
			_drive.setRightSideInverted(false); // do not change this
		/* Configure output direction */
		//_leftMaster.setInverted(false);
		//_rightMaster.setInverted(true);
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
		
	}
	
	@Override
	public void teleopPeriodic() {		
		/* Gamepad processing */
		double forward = 1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		double liftup = _game2.getY();
		double driveLift = _game2.getRawAxis(5);
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		//_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		//_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		//_drive.arcadeDrive(-forward, turn);
		climbFront.set(liftup);
		climbBack.set(liftup*.60);
		climbDrive.set(-driveLift);
		elevatorDrive.set(forward);

		sampleEncoder.setMaxPeriod(.1);
		sampleEncoder.setMinRate(10);
		sampleEncoder.setDistancePerPulse(5);
		sampleEncoder.setReverseDirection(true);
		sampleEncoder.setSamplesToAverage(7);

		int count1 = sampleEncoder.get();
		double distance = sampleEncoder.getRaw();
		double distance2 = sampleEncoder.getDistance();
		double period = sampleEncoder.getPeriod();
		double rate = sampleEncoder.getRate();
		boolean direction = sampleEncoder.getDirection();
		boolean stopped = sampleEncoder.getStopped();

		//SmartDashboard.putData('Test count',count1);
	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.25) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.25)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}
