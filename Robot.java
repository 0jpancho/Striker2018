/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team101.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot 
{	
	WPI_TalonSRX rightSlave, leftSlave, rightMaster, leftMaster, liftMaster, liftSlave;	
	Talon climberOne, climberTwo;
	
	Joystick driverOne, driverTwo, operator;
	
	DigitalInput limitSwitchTop;
	
	boolean talonsBraked;
	
	boolean toggleArms;
	boolean togglePusher;
	
	AHRS navX;
	
	DoubleSolenoid cubeArms, cubePusher;
	
	@SuppressWarnings("deprecation")
	NetworkTable sdTable = NetworkTable.getTable("SmartDashboard");
	
	final double TURN_ANGLE_SPEED = 0.25;
	
	SendableChooser<String> chosenPosition = new SendableChooser<String>();
	
	final double wheelCircumference = 6 * Math.PI;
	final double ppr = 4096;
	
	final double inchesPerCount = wheelCircumference / ppr;
	final double countsPerInch = 1 / inchesPerCount;
	
	Timer autonTimer;
	
	@Override
	public void robotInit()
	{	
		rightSlave = new WPI_TalonSRX(8);
		rightMaster = new WPI_TalonSRX(1);
		
		rightMaster.setSafetyEnabled(false);
		rightSlave.setSafetyEnabled(false);
		
		leftSlave = new WPI_TalonSRX(2);
		leftMaster = new WPI_TalonSRX(3);
		
		leftMaster.setSafetyEnabled(false);
		leftSlave.setSafetyEnabled(false);
		
		rightSlave.set(ControlMode.Follower, 1);
		leftSlave.set(ControlMode.Follower, 3);
		
		liftMaster = new WPI_TalonSRX(4);
		liftMaster.setNeutralMode(NeutralMode.Brake);
		liftMaster.setInverted(false);
		
		liftSlave = new WPI_TalonSRX(5);
		liftSlave.setNeutralMode(NeutralMode.Brake);
		
		liftSlave.setInverted(false);
		liftSlave.set(ControlMode.Follower, 4);
	
		limitSwitchTop = new DigitalInput(0);
		
		cubeArms = new DoubleSolenoid(0, 1);
		cubePusher = new DoubleSolenoid(2, 3);
		
		driverOne = new Joystick(0);
		driverTwo = new Joystick(1);
		operator = new Joystick(2);
		
		leftSlave.setInverted(true);
		leftMaster.setInverted(true);
		
		navX = new AHRS(SerialPort.Port.kUSB);
		
		chosenPosition.addObject("Left", "Left");
		chosenPosition.addObject("Middle", "Middle");
		chosenPosition.addObject("Right", "Right");
		SmartDashboard.putData("chosenPosition", chosenPosition);
		
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);	
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
		
		leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		
		leftMaster.setSelectedSensorPosition(0, 0, 20);
		rightMaster.setSelectedSensorPosition(0, 0, 20);
		
		leftMaster.setSensorPhase(false);
		rightMaster.setSensorPhase(false);
		
		leftMaster.configNominalOutputForward(0, 20);
		leftMaster.configNominalOutputReverse(0, 20);
		
		leftMaster.configPeakOutputForward(1, 20);
		leftMaster.configPeakOutputReverse(-1, 20);
		
		rightMaster.configNominalOutputForward(0, 20);
		rightMaster.configNominalOutputReverse(0, 20);
		
		rightMaster.configPeakOutputForward(1, 20);
		rightMaster.configPeakOutputReverse(-1, 20);
		
		leftSlave.configPeakOutputForward(1, 20);
		leftSlave.configPeakOutputReverse(-1, 20);
		
		rightSlave.configPeakOutputForward(1, 20);
		rightSlave.configPeakOutputReverse(-1, 20);
	}
	
	@SuppressWarnings("deprecation")
	public void disabledPeriodic() {
		
		robotTelemetry();
	}
	
	@Override
	public void autonomousInit() 
	{
		toggleBrakeMode(true);
		navX.zeroYaw();
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		//Possibility 1: Left Left Left, start left
		if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Left"))
		{	
			liftLift(2, true);
			driveDistance(130, 0.5);
			turnAngle(30);
			liftLift(2, true);
			togglePusher(true);
			toggleArms(false);
			togglePusher(false);
		}	
		
		//Possibility 2: Left Left Left, start middle
		else if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Middle"))
		{
			liftLift(2, true);
		}
		
		//Possibility 3: Left Left Left, start right
		else if(gameData.equals("LLL") && chosenPosition.getSelected().equals("Right"))
		{
			liftLift(2, false);
		}
		
		//Possibility 4: Left Right Left, start left
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Left"))
		{
			
		}
		
		//Possibility 5: Left Right Left, start middle
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Middle"))
		{
			
		}
		
		//Possibility 6: Left Right Left, start right
		else if(gameData.equals("LRL") && chosenPosition.getSelected().equals("Right"))
		{
			
		}
		
		//Possibility 7: Right Left Right, start left
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Left"))
		{
			
		}
		
		//Possibility 8: Right Left Right, start middle
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Middle"))
		{
			
		}
		
		//Possibility 9: Right Left Right, start right
		else if(gameData.equals("RLR") && chosenPosition.getSelected().equals("Right"))
		{
			
		}
		
		//Possibility 10: Right Right Right, start left
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Left"))
		{
			
		}
		
		//Possibility 11: Right Right Right, start middle
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Middle"))
		{
			
		}

		//Possibility 12: Right Right Right, start right
		else if(gameData.equals("RRR") && chosenPosition.getSelected().equals("Right"))
		{
			
		}
	}
	
	@Override
	public void autonomousPeriodic()
	{

	}
	@Override
	public void teleopPeriodic()
	{	
		leftMaster.set(ControlMode.PercentOutput, driverOne.getY());
		rightMaster.set(ControlMode.PercentOutput, driverTwo.getY());
		
		if (driverOne.getTrigger() && driverTwo.getTrigger()) 
		{
			toggleBrakeMode(true);
		}
		
		else 
		{
			toggleBrakeMode(false);
		}
		
		if(limitSwitchTop.get() == true && operator.getY() > 0)
		{
			liftMaster.set(ControlMode.Disabled, 0);
		}
		else 
		{
			liftMaster.set(ControlMode.PercentOutput, operator.getY());
		}
	
		if (operator.getTrigger() && !toggleArms)
		{
			if(cubeArms.get() == Value.kReverse || cubeArms.get() == Value.kOff)
				cubeArms.set(Value.kForward);
			
			else if(cubeArms.get() == Value.kForward)
				cubeArms.set(Value.kReverse);
				
			toggleArms = true;
		}
		
		else if(!operator.getTrigger())
			toggleArms = false;
		
		if (operator.getRawButton(3) && !togglePusher)
		{
			if(cubePusher.get() == Value.kReverse || cubePusher.get() == Value.kOff)
				cubePusher.set(Value.kForward);
			
			else if(cubePusher.get() == Value.kForward)
				cubePusher.set(Value.kReverse);
			
			togglePusher = true;
		}
		
		else if (!operator.getRawButton(3))
			togglePusher = false;
		
		if (operator.getRawButton(2))
		{
			climberOne.set(1);
			climberTwo.set(1);
		}
		
		climberOne.set(0);
		climberTwo.set(0);
		
		robotTelemetry();
	}

	@Override
	public void testPeriodic()
	{
	
	}

	@SuppressWarnings("deprecation")
	public void turnAngle(float degrees)
	{
		//Turn Counter-clockwise
		while (navX.getYaw() < degrees) 
		{
			robotTelemetry();
			
			leftSlave.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
			leftMaster.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
			rightSlave.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			rightMaster.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
		}
		
		leftSlave.set(ControlMode.PercentOutput, 0);
		leftMaster.set(ControlMode.PercentOutput, 0);
		rightSlave.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);

		//Turn Clockwise
		while (navX.getYaw() > degrees)
		{
			robotTelemetry();
			
			leftSlave.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			leftMaster.set(ControlMode.PercentOutput, TURN_ANGLE_SPEED);
			rightSlave.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
			rightMaster.set(ControlMode.PercentOutput, -TURN_ANGLE_SPEED);
		}
		
		leftSlave.set(ControlMode.PercentOutput, 0);
		leftMaster.set(ControlMode.PercentOutput, 0);
		rightSlave.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public void toggleBrakeMode(boolean toggleBrake)
	{
		if(toggleBrake)
		{
			leftSlave.setNeutralMode(NeutralMode.Brake);
			leftMaster.setNeutralMode(NeutralMode.Brake);
			
			rightSlave.setNeutralMode(NeutralMode.Brake);
			rightMaster.setNeutralMode(NeutralMode.Brake);
		}
		
		else if (!toggleBrake)
		{
			leftSlave.setNeutralMode(NeutralMode.Coast);
			leftMaster.setNeutralMode(NeutralMode.Coast);
			
			rightSlave.setNeutralMode(NeutralMode.Coast);
			rightMaster.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public void resetDriveEncoders()
	{
		leftMaster.setSelectedSensorPosition(0, 0, 20);
		rightMaster.setSelectedSensorPosition(0, 0, 20);
		
		leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
	}
	
	public void driveDistance(double inches, double speed)
	{
		double targetCounts = countsPerInch * inches;
		
		resetDriveEncoders();
		
		System.out.println(targetCounts);
		while (this.isEnabled())
		{	
			robotTelemetry();
			
			if (targetCounts > leftMaster.getSensorCollection().getQuadraturePosition())
			{
				leftMaster.set(ControlMode.PercentOutput, -speed);
			}
			
			if (targetCounts > rightMaster.getSensorCollection().getQuadraturePosition())
			{
				rightMaster.set(ControlMode.PercentOutput, -speed);
			}
			
			if(targetCounts < leftMaster.getSensorCollection().getQuadraturePosition() 
					|| targetCounts < rightMaster.getSensorCollection().getQuadraturePosition())
				break;
		}
		
		leftMaster.set(ControlMode.Disabled, 0);
		rightMaster.set(ControlMode.Disabled, 0);
	}
	
	public void liftLift(double duration, boolean upward)
	{
		autonTimer.start();
		
		double speed = 0.5;
		
		if (!upward)
			speed = -0.5;
		
		
		while (autonTimer.get() >= 0 && autonTimer.get() < duration && !limitSwitchTop.get())
		{
			liftMaster.set(ControlMode.PercentOutput, speed);
		}
		
		liftMaster.set(ControlMode.Disabled, 0);
		
		autonTimer.stop();
		autonTimer.reset();
	}
	
	public void toggleArms(boolean armToggle)
	{
		if (armToggle)
		{
			cubeArms.set(Value.kForward);
		}
		
		else if (!armToggle) 
		{
			cubeArms.set(Value.kReverse);
		}
	}
	
	public void togglePusher (boolean pusherToggle)
	{
		if (pusherToggle)
		{
			cubePusher.set(Value.kForward);
		}
		
		else if (!pusherToggle)
		{
			cubePusher.set(Value.kReverse);
		}
	}
	
	public void robotTelemetry() 
	{	
		sdTable.putDouble("OrIeNtAtIoN", navX.getYaw());
		sdTable.putBoolean("NavX Connected?", navX.isConnected());
		sdTable.putBoolean("NavX Calibrating?", navX.isCalibrating());
		
		sdTable.putBoolean("Talons Braked?", talonsBraked);
	
		SmartDashboard.putNumber("Left Master Enc Counts", leftMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Master Enc Counts", rightMaster.getSelectedSensorPosition(0));
		
		SmartDashboard.putBoolean("Top Limit Switch", limitSwitchTop.get());
		
		SmartDashboard.putBoolean("Pusher Toggled?", cubePusher.get() == Value.kForward);
		SmartDashboard.putBoolean("Arms Toggled?", cubeArms.get() == Value.kForward);
	}
}