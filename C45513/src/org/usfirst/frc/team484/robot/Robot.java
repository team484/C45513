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
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team484.robot.MatchData.GameFeature;
import org.usfirst.frc.team484.robot.MatchData.OwnedSide;
import org.usfirst.frc.team484.robot.commands.auto.AutoDoNothing;
import org.usfirst.frc.team484.robot.commands.auto.CrossAutoLine;
import org.usfirst.frc.team484.robot.commands.auto.StraightThenLeftToScale;
import org.usfirst.frc.team484.robot.commands.auto.StraightThenLeftToSwitch;
import org.usfirst.frc.team484.robot.commands.auto.StraightThenRightToScale;
import org.usfirst.frc.team484.robot.commands.auto.StraightThenRightToSwitch;
import org.usfirst.frc.team484.robot.commands.auto.StraightToSwitch;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;
import org.usfirst.frc.team484.robot.subsystems.GrabberAngleSub;
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
	public static final GrabberAngleSub grabberAngleSub = new GrabberAngleSub();
	
	//-----Creates Operator Interface-----
	public static OI oi;
	
	//-----Choosers for Auto Mode-----
	private static Command autonomousCommand;
	private static Command delayCommand;
	private static final SendableChooser<Command> llChooser = new SendableChooser<>();
	private static final SendableChooser<Command> lrChooser = new SendableChooser<>();
	private static final SendableChooser<Command> rlChooser = new SendableChooser<>();
	private static final SendableChooser<Command> rrChooser = new SendableChooser<>();
	private static boolean hasAutoCommandStarted = false;
	

	@Override
	public void robotInit() {
		setPeriod(RobotSettings.ROBOT_UPDATE_RATE);
		oi = new OI();
		
		llChooser.setName("Switch Left Scale Left");
		llChooser.addDefault("Do Nothing", new AutoDoNothing());
		llChooser.addObject("Switch Front", new StraightToSwitch());
		llChooser.addObject("Switch Side", new StraightThenRightToSwitch());
		llChooser.addObject("Scale", new StraightThenRightToScale());
		llChooser.addObject("Cross Auto Line", new CrossAutoLine());
		lrChooser.setName("Switch Left Scale Right");
		lrChooser.addDefault("Do Nothing", new AutoDoNothing());
		lrChooser.addObject("Switch Front", new StraightToSwitch());
		lrChooser.addObject("Switch Side", new StraightThenRightToSwitch());
		lrChooser.addObject("Cross Auto Line", new CrossAutoLine());
		rlChooser.setName("Switch Right Scale Left");
		rlChooser.addDefault("Do Nothing", new AutoDoNothing());
		rlChooser.addObject("Switch Front", new StraightToSwitch());
		rlChooser.addObject("Switch Side", new StraightThenLeftToSwitch());
		rlChooser.addObject("Cross Auto Line", new CrossAutoLine());
		rrChooser.setName("Switch Right Scale Right");
		rrChooser.addDefault("Do Nothing", new AutoDoNothing());
		rrChooser.addObject("Switch Front", new StraightToSwitch());
		rrChooser.addObject("Switch Side", new StraightThenLeftToSwitch());
		rrChooser.addObject("Scale", new StraightThenLeftToScale());
		rrChooser.addObject("Cross Auto Line", new CrossAutoLine());
		
		SmartDashboard.putNumber("Delay", 0);
		SmartDashboard.putNumber("Delay Set To", SmartDashboard.getNumber("Delay", 0));
		
		SmartDashboard.putData(llChooser);
		SmartDashboard.putData(lrChooser);
		SmartDashboard.putData(rlChooser);
		SmartDashboard.putData(rrChooser);
		
		RobotIO.watchdog.enable();
	}

	@Override
	public void disabledInit() {
		RobotIO.logger.disable();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Delay Set To", SmartDashboard.getNumber("Delay", 0));
	}

	@Override
	public void autonomousInit() {
		RobotIO.logger.enable();
		hasAutoCommandStarted = false;
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
		
		double delay = SmartDashboard.getNumber("Delay", 0);
		delayCommand = new WaitCommand("Auto Delay", delay);
		delayCommand.start();
		
		RobotIO.logger.enable();
	}

	@Override
	public void autonomousPeriodic() {
		if (delayCommand.isCompleted() && autonomousCommand != null && !hasAutoCommandStarted) {
			hasAutoCommandStarted = true;
			autonomousCommand.start();
		}
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (delayCommand != null) delayCommand.cancel();
		if (autonomousCommand != null) autonomousCommand.cancel();
		
		RobotIO.logger.enable();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
