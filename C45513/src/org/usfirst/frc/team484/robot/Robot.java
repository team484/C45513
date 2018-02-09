/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.usfirst.frc.team484.robot.MatchData.GameFeature;
import org.usfirst.frc.team484.robot.MatchData.OwnedSide;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;
import org.usfirst.frc.team484.robot.subsystems.GrabberSub;
import org.usfirst.frc.team484.robot.subsystems.ShifterSub;

public class Robot extends TimedRobot {
	//-----Creates all robot objects-----
	public static final RobotIO robotIO = new RobotIO();
	
	//-----Creates all subsystems-----
	public static final DriveSub driveSub = new DriveSub();
	public static final ElevatorSub elevatorSub = new ElevatorSub();
	public static final GrabberSub grabberSub = new GrabberSub();
	public static final ShifterSub shifterSub = new ShifterSub();
	
	//-----Creates Operator Interface-----
	public static OI oi;
	
	//-----Choosers for Auto Mode-----
	Command autonomousCommand;
	SendableChooser<Command> llChooser = new SendableChooser<>();
	SendableChooser<Command> lrChooser = new SendableChooser<>();
	SendableChooser<Command> rlChooser = new SendableChooser<>();
	SendableChooser<Command> rrChooser = new SendableChooser<>();
	

	@Override
	public void robotInit() {
		setPeriod(RobotSettings.ROBOT_UPDATE_RATE);
		oi = new OI();
		
		llChooser.setName("Switch Left Scale Left");
		lrChooser.setName("Switch Left Scale Right");
		rlChooser.setName("Switch Right Scale Left");
		rrChooser.setName("Switch Right Scale Right");
	}

	@Override
	public void disabledInit() {
		if (RobotIO.logger.isAlive()) RobotIO.logger.interrupt();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		OwnedSide switchState = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		OwnedSide scaleState = MatchData.getOwnedSide(GameFeature.SCALE);
		if (switchState == OwnedSide.LEFT) {
			if (scaleState == OwnedSide.LEFT) {
				autonomousCommand = llChooser.getSelected();
			} else if (scaleState == OwnedSide.RIGHT) {
				autonomousCommand = lrChooser.getSelected();
			} else {
				System.err.println("Could not get field state");
			}
		} else if (switchState == MatchData.OwnedSide.RIGHT) {
			if (scaleState == OwnedSide.LEFT) {
				autonomousCommand = rlChooser.getSelected();
			} else if (scaleState == OwnedSide.RIGHT) {
				autonomousCommand = rrChooser.getSelected();
			} else {
				System.err.println("Could not get field state");
			}
		} else {
			System.err.println("Could not get field state");
		}
		
		if (autonomousCommand != null) autonomousCommand.start();
		
		if (!RobotIO.logger.isAlive()) RobotIO.logger.start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) autonomousCommand.cancel();
		
		if (!RobotIO.logger.isAlive()) RobotIO.logger.start();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
