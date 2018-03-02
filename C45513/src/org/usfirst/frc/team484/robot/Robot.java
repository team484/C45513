/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleAngledFromP1;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleAngledFromP5;
import org.usfirst.frc.team484.robot.commands.auto.LeftSwitchFromP3;
import org.usfirst.frc.team484.robot.commands.auto.LeftSwitchFromP3V2;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleAngledFromP1;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleAngledFromP5;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleFromP1;
import org.usfirst.frc.team484.robot.commands.auto.RightSwitchFromP3;
import org.usfirst.frc.team484.robot.commands.auto.RightSwitchFromP3V2;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleFromP5;
import org.usfirst.frc.team484.robot.commands.auto.SideOfRightSwitchFromP5;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleFromP1;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleFromP5;
import org.usfirst.frc.team484.robot.commands.auto.SideOfLeftSwitchFromP1;
import org.usfirst.frc.team484.robot.commands.auto.StraightToSwitch;
import org.usfirst.frc.team484.robot.subsystems.ClimberSub;
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
	public static final ClimberSub climberSub = new ClimberSub();

	//-----Creates Operator Interface-----
	public static OI oi;

	//-----Choosers for Auto Mode-----
	private static Command autonomousCommand;
	private static Command delayCommand;
	private static SendableChooser<Command> llChooser = new SendableChooser<>();
	private static SendableChooser<Command> lrChooser = new SendableChooser<>();
	private static SendableChooser<Command> rlChooser = new SendableChooser<>();
	private static SendableChooser<Command> rrChooser = new SendableChooser<>();
	private static SendableChooser<Integer> position = new SendableChooser<>();
	private static boolean hasAutoCommandStarted = false;

	private static boolean isCameraServerUp = false;


	@Override
	public void robotInit() {
		try {
			setPeriod(RobotSettings.ROBOT_UPDATE_RATE);
			oi = new OI();
			position.addDefault("(1) Far Left",1);
			position.addObject("(2) Left",2);
			position.addObject("(3) Center",3);
			position.addObject("(4) Right",4);
			position.addObject("(5) Far Right",5);
			SmartDashboard.putData("Field Position", position);
			SmartDashboard.setDefaultNumber("Delay", 0);
		} catch (Throwable t) {
			t.printStackTrace();
		}

		enableCameraServer();
	}

	@Override
	public void disabledInit() {
		try {
			RobotIO.logger.endLogging();
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			SmartDashboard.putNumber("Delay Set To", SmartDashboard.getNumber("Delay", 0));
			Scheduler.getInstance().run();
			updateChoosers();
			enableCameraServer(); //Will try to enable server if it has not started up yet.
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void autonomousInit() {
		try {
			RobotIO.setVoltageComp(true);
			RobotIO.elevatorEncoder.reset();

			hasAutoCommandStarted = false;
			autonomousCommand = null;
			updateAutoSelection();

			double delay = SmartDashboard.getNumber("Delay", 0);
			delayCommand = new WaitCommand("Auto Delay", delay);
			delayCommand.start();

			RobotIO.logger.startLogging("auto");

		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {
			// People have reported that the game specific message does not always come in before
			//autonomous starts. This will continue updating until the message has come
			if (autonomousCommand == null) {
				updateAutoSelection();
			}
			if (delayCommand.isCompleted() && autonomousCommand != null && !hasAutoCommandStarted) {
				hasAutoCommandStarted = true;
				autonomousCommand.start();
			}
			Scheduler.getInstance().run();
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void teleopInit() {
		try {
			RobotIO.setVoltageComp(false);
			if (delayCommand != null) delayCommand.cancel();
			if (autonomousCommand != null) autonomousCommand.cancel();

			RobotIO.logger.startLogging("tele");
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void teleopPeriodic() {
		try {
			Scheduler.getInstance().run();
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Pressure", RobotIO.getAirPressure());
		SmartDashboard.putNumber("Voltage", RobotIO.pdp.getVoltage());
	}

	/**
	 * Sets the auto mode based on the GameSpecificMessage.
	 */
	private static void updateAutoSelection() {
		if (DriverStation.getInstance().getGameSpecificMessage() == null) return;
		if (DriverStation.getInstance().getGameSpecificMessage().length() < 3) return;
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

	}

	private static void enableCameraServer() {
		if (!isCameraServerUp) {
			try {
				UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
				camera.setExposureAuto();
				camera.setWhiteBalanceAuto();
				camera.setFPS(120);
				camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
				isCameraServerUp = true;
			} catch (Throwable t) {
				t.printStackTrace();
			}
		}
	}
	
	private static int oldPos = 1;
	/**
	 * Update the SendableChoosers based on field position
	 */
	private static void updateChoosers() {
		int pos = position.getSelected();
		if (pos == oldPos) return;
		oldPos = pos;
		SmartDashboard.delete("Left Switch Left Scale");
		SmartDashboard.delete("Left Switch Right Scale");
		SmartDashboard.delete("Right Switch Left Scale");
		SmartDashboard.delete("Right Switch Right Scale");
		llChooser = new SendableChooser<Command>();
		lrChooser = new SendableChooser<Command>();
		rlChooser = new SendableChooser<Command>();
		rrChooser = new SendableChooser<Command>();
		
		llChooser.addDefault("Do Nothing", new AutoDoNothing());
		lrChooser.addDefault("Do Nothing", new AutoDoNothing());
		rlChooser.addDefault("Do Nothing", new AutoDoNothing());
		rrChooser.addDefault("Do Nothing", new AutoDoNothing());
		
		llChooser.addObject("Cross Auto Line", new CrossAutoLine());
		lrChooser.addObject("Cross Auto Line", new CrossAutoLine());
		rlChooser.addObject("Cross Auto Line", new CrossAutoLine());
		rrChooser.addObject("Cross Auto Line", new CrossAutoLine());
		
		switch(pos) {
		case 1:
			llChooser.addObject("Switch Side", new SideOfLeftSwitchFromP1());
			lrChooser.addObject("Switch Side", new SideOfLeftSwitchFromP1());

			llChooser.addObject("Scale Side", new LeftScaleFromP1());
			lrChooser.addObject("Scale Side", new RightScaleFromP1());
			rlChooser.addObject("Scale Side", new LeftScaleFromP1());
			rrChooser.addObject("Scale Side", new RightScaleFromP1());

			llChooser.addObject("Scale Corner", new LeftScaleAngledFromP1());
			lrChooser.addObject("Scale Corner", new RightScaleAngledFromP1());
			rlChooser.addObject("Scale Corner", new LeftScaleAngledFromP1());
			rrChooser.addObject("Scale Corner", new RightScaleAngledFromP1());
			break;
		case 2:

			llChooser.addObject("Switch Front", new StraightToSwitch());
			lrChooser.addObject("Switch Front", new StraightToSwitch());
			break;
		case 3:
			llChooser.addObject("Center to Left Switch", new LeftSwitchFromP3());
			lrChooser.addObject("Center to Left Switch", new LeftSwitchFromP3());
			rlChooser.addObject("Center to Right Switch", new RightSwitchFromP3());
			rrChooser.addObject("Center to Right Switch", new RightSwitchFromP3());
			llChooser.addObject("Center to Left Switch (V2)", new LeftSwitchFromP3V2());
			lrChooser.addObject("Center to Left Switch (V2)", new LeftSwitchFromP3V2());
			rlChooser.addObject("Center to Right Switch (V2)", new RightSwitchFromP3V2());
			rrChooser.addObject("Center to Right Switch (V2)", new RightSwitchFromP3V2());
			break;
		case 4:
			rlChooser.addObject("Switch Front", new StraightToSwitch());
			rrChooser.addObject("Switch Front", new StraightToSwitch());
			break;
		case 5:
			rlChooser.addObject("Switch Side", new SideOfRightSwitchFromP5());
			rrChooser.addObject("Switch Side", new SideOfRightSwitchFromP5());
			
			llChooser.addObject("Scale Side", new LeftScaleFromP5());
			lrChooser.addObject("Scale Side", new RightScaleFromP5());
			rlChooser.addObject("Scale Side", new LeftScaleFromP5());
			rrChooser.addObject("Scale Side", new RightScaleFromP5());

			llChooser.addObject("Scale Corner", new LeftScaleAngledFromP5());
			lrChooser.addObject("Scale Corner", new RightScaleAngledFromP5());
			rlChooser.addObject("Scale Corner", new LeftScaleAngledFromP5());
			rrChooser.addObject("Scale Corner", new RightScaleAngledFromP5());
			break;
		default:
			break;
		}
		SmartDashboard.putData("Left Switch Left Scale", llChooser);
		SmartDashboard.putData("Left Switch Right Scale", lrChooser);
		SmartDashboard.putData("Right Switch Left Scale", rlChooser);
		SmartDashboard.putData("Right Switch Right Scale", rrChooser);
	}
}
