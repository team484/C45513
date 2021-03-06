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
import edu.wpi.first.wpilibj.vision.VisionThread;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team484.robot.MatchData.GameFeature;
import org.usfirst.frc.team484.robot.MatchData.OwnedSide;
import org.usfirst.frc.team484.robot.commands.auto.AutoDoNothing;
import org.usfirst.frc.team484.robot.commands.auto.CrossAutoLine;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleAndSwitchFromP1;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleFromP5;
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleFrontFromP1;
<<<<<<< HEAD
import org.usfirst.frc.team484.robot.commands.auto.LeftScaleSideFromP1;
=======
>>>>>>> branch 'master' of https://github.com/team484/C45513.git
import org.usfirst.frc.team484.robot.commands.auto.LeftSwitchFromP3;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleAndSwitchFromP5;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleFromP1;
import org.usfirst.frc.team484.robot.commands.auto.RightScaleFrontFromP5;
<<<<<<< HEAD
import org.usfirst.frc.team484.robot.commands.auto.RightScaleSideFromP5;
=======
>>>>>>> branch 'master' of https://github.com/team484/C45513.git
import org.usfirst.frc.team484.robot.commands.auto.RightSwitchFromP3;
import org.usfirst.frc.team484.robot.commands.auto.SideOfRightSwitchFromP5;
import org.usfirst.frc.team484.robot.commands.auto.SideOfLeftSwitchFromP1;
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

	//-----Choosers for Auto Mode-----
	private static Command autonomousCommand;
	private static Command delayCommand;
	private static SendableChooser<Command> llChooser = new SendableChooser<>();
	private static SendableChooser<Command> lrChooser = new SendableChooser<>();
	private static SendableChooser<Command> rlChooser = new SendableChooser<>();
	private static SendableChooser<Command> rrChooser = new SendableChooser<>();
	private static SendableChooser<Integer> position = new SendableChooser<>();
	private static boolean hasAutoCommandStarted = false;

	//-----Camera and computer vision objects-----
	private static boolean isCameraServerUp = false;
	public static VisionThread visionThread;
	public static double visionCubeAngle = 0.0; //The angle the robot must turn to face the cube
	public static double visionCubeY = 0.0; //The distance (in inches) to the cube
	public static double visionCubeX = 0.0; //The translation (in inches) to the cube
	public static final Object imgLock = new Object(); //Obj to synchronize to for thread safety
	public static long lastVisionUpdate = 0; //Last timestamp the results were updated


	@Override
	public void robotInit() {
		try {
			setPeriod(RobotSettings.TIME_STEP);
			OI.setupOI();
			position.addDefault("(1) Far Left",1);
			position.addObject("(2) Left",2);
			position.addObject("(3) Center",3);
			position.addObject("(4) Right",4);
			position.addObject("(5) Far Right",5);
			SmartDashboard.putData("Field Position", position);
			SmartDashboard.setDefaultNumber("Delay", 0);
			updateChoosers();
		} catch (Throwable t) {
			t.printStackTrace();
		}

		//True will automatically recalculate every trajectory below.
		GenerateTrajectory.forceRegen = RobotSettings.FORCE_PATH_REGEN;
		
		GenerateTrajectory.execute("RightScaleFromP1",
				new Waypoint( 47.3 	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint( 47.3	, 167.9	, Pathfinder.d2r( 90)),
<<<<<<< HEAD
				new Waypoint(121.8	, 233.6 , Pathfinder.d2r(  0)),
				new Waypoint(220.0	, 233.6	, Pathfinder.d2r(  0)),
				new Waypoint(279.6	, 282.0	, Pathfinder.d2r( 90)));
=======
				new Waypoint(121.8	, 240.0 , Pathfinder.d2r(  0)),
				new Waypoint(220.0	, 240.0	, Pathfinder.d2r(  0)),
				new Waypoint(255.0	, 285.0	, Pathfinder.d2r( 90)));
>>>>>>> branch 'master' of https://github.com/team484/C45513.git
		
		GenerateTrajectory.execute("LeftScaleFrontFromP1",
				new Waypoint( 47.3	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint( 47.3	, 167.9	, Pathfinder.d2r( 90)),
				new Waypoint( 101.0	, 283.0	, Pathfinder.d2r( 90)));
		
		GenerateTrajectory.execute("LeftSwitchFromP1",
				new Waypoint( 47.3 	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint( 47.3	, 110.0	, Pathfinder.d2r( 90)),
				new Waypoint( 87.0	, 157.0	, Pathfinder.d2r(  0)));
		
		GenerateTrajectory.execute("LeftSwitchFromP3",
				new Waypoint(167.3	,  19.5 , Pathfinder.d2r( 90)),
				new Waypoint(120	, 121.3	, Pathfinder.d2r( 90)));
		GenerateTrajectory.execute("RightSwitchFromP3",
				new Waypoint(167.3	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint(210.3	, 121.3	, Pathfinder.d2r( 90)));
		GenerateTrajectory.execute("LeftScaleFromP5",
				new Waypoint(277.3	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint(277.3	, 167.9	, Pathfinder.d2r( 90)),
<<<<<<< HEAD
				new Waypoint(202.3	, 233.6	, Pathfinder.d2r(180)),
				new Waypoint(104.0	, 233.6	, Pathfinder.d2r(180)),
				new Waypoint( 54.4	, 282.0	, Pathfinder.d2r( 90)));
=======
				new Waypoint(202.3	, 240.0	, Pathfinder.d2r(180)),
				new Waypoint(104.0	, 240.0	, Pathfinder.d2r(180)),
				new Waypoint( 54.6+15.0	, 285.0	, Pathfinder.d2r( 90)));
>>>>>>> branch 'master' of https://github.com/team484/C45513.git
		
		GenerateTrajectory.execute("RightScaleFrontFromP5",
				new Waypoint(277.3	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint(277.3	, 167.9	, Pathfinder.d2r( 90)),
				new Waypoint(226.6	, 283.0	, Pathfinder.d2r( 90)));
		
		GenerateTrajectory.execute("RightSwitchFromP5",
				new Waypoint(277.3	,  19.5	, Pathfinder.d2r( 90)),
				new Waypoint(277.3	, 110.0	, Pathfinder.d2r( 90)),
				new Waypoint(227.6	, 157.0	, Pathfinder.d2r(180)));
		
		enableCameraServer();
		//visionThread.start(); //Uncomment for vision testing
		RobotIO.logger.log("DS", DriverStation.getInstance());
		RobotIO.logger.log("PDP", RobotIO.pdp);
		RobotIO.logger.log("Left", RobotIO.leftDriveMotors);
		RobotIO.logger.log("Right", RobotIO.rightDriveMotors);
		RobotIO.logger.log("Left Enc", RobotIO.leftEncoder);
		RobotIO.logger.log("Right Enc", RobotIO.rightEncoder);
		RobotIO.logger.log("IR", RobotIO.irSensor);
		RobotIO.logger.log("Pressure", RobotIO.pressureSensor);
		RobotIO.logger.log("ElevDown", RobotIO.elevatorDownDI);
		RobotIO.logger.log("ElevUp", RobotIO.elevatorUpDI);
		RobotIO.logger.log("GrabberDow", RobotIO.grabberAngleDownDI);
		RobotIO.logger.log("ElevMotor", RobotIO.elevatorMotorL1);
		RobotIO.logger.log("Shifter", RobotIO.shifterSolenoid);
		RobotIO.logger.log("Grabber", RobotIO.grabberSolenoid);
	}

	@Override
	public void disabledInit() {
		try {
			RobotIO.logger.endLogging();
			if (visionThread != null) {
				if (visionThread.isAlive()) {
					visionThread.interrupt(); //Comment out for vision testing
				}
			}
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

			hasAutoCommandStarted = false;
			autonomousCommand = null;
			updateAutoSelection();

			double delay = SmartDashboard.getNumber("Delay", 0);
			delayCommand = new WaitCommand("Auto Delay", delay);
			delayCommand.start();

			//RobotIO.logger.startLogging("auto");
			visionThread.start();
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
			OI.setupOI(); // Runs this in case a joystick was plugged in after robot init
			RobotIO.logger.startLogging("tele");
			if (visionThread != null) {
				if (visionThread.isAlive()) {
					visionThread.interrupt();
				}
			}
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
		SmartDashboard.putNumber("Elevator Height", RobotIO.getElevatorHeight());
		//SmartDashboard.putNumber("CubeAngle", visionCubeAngle);
		//SmartDashboard.putNumber("CubeX", visionCubeX);
		//SmartDashboard.putNumber("CubeY", visionCubeY);
		//SmartDashboard.putNumber("Vision Period", lastVisionUpdate);
		//SmartDashboard.putNumber("Elevator Height", RobotIO.getElevatorHeight());
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

	//private static Point cubeP1 = new Point();
	//private static Point cubeP2 = new Point();
	/**
	 * Starts the camera server and initializes the vision thread object.
	 */
	private static void enableCameraServer() {
		if (!isCameraServerUp) {
			try {
				UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
				camera.setExposureManual(2);
				camera.setWhiteBalanceAuto();
				camera.setFPS(30);
				camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
				isCameraServerUp = true;
				/*new Thread(() -> {	                
	                CvSink cvSink = CameraServer.getInstance().getVideo();
	                CvSource outputStream = CameraServer.getInstance().putVideo("camera stream", 320, 240);
	                
	                Mat source = new Mat();
	                
	                while(!Thread.interrupted()) {
	                    cvSink.grabFrame(source);
	                    Imgproc.rectangle(source, cubeP1, cubeP2, new Scalar(0.0, 255.0, 0.0), 1);
	                    outputStream.putFrame(source);
	                }
	            }).start();*/
				//A lambda expression to make Max happy
				visionThread = new VisionThread(camera, new CubeVisionPipeline(), pipeline -> {
					if (!pipeline.filterContoursOutput().isEmpty()) {
						//Find the largest contour
						double maxSize = 0;
						Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
						for (MatOfPoint contour : pipeline.filterContoursOutput()) {
							double size = Imgproc.contourArea(contour, true);
							if (size > maxSize) {
								maxSize = size;
								r = Imgproc.boundingRect(contour);
							}
						}
						//cubeP1 = r.tl();
						//cubeP2 = r.br();
						synchronized (imgLock) {
							double centerX = r.x + (r.width / 2);
							double centerY = r.y + (r.height / 2);
							final double radsPerPixel = 0.00736;
							double visionAngleY = -0.8726643268 - (centerY - 120.0) * radsPerPixel;
							double yzDistance = -(38.0-11.0)/Math.sin(visionAngleY);
							double visionAngleX = (160 - centerX) * radsPerPixel;
							double xyzDistance = yzDistance / Math.cos(visionAngleX);
							//double zDist = xyzDistance * Math.cos(visionAngleX) * Math.sin(visionAngleY);
							double xDist = xyzDistance * Math.sin(visionAngleX) * Math.sin(visionAngleY);
							double yDist = xyzDistance * Math.cos(visionAngleY);
							double yActual = 1.0353 * yDist + 17.748;
							double xActual = 0.75 + xDist * 0.8889;
							double xyAngle = Math.toDegrees(Math.atan(-xActual/yActual));
							visionCubeX = xActual;
							visionCubeY = yActual;
							visionCubeAngle = xyAngle;
							lastVisionUpdate = System.currentTimeMillis();
						}
					}
				});
			} catch (Throwable t) {
				t.printStackTrace();
			}
		}
	}

	private static int oldPos = -1;
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

			lrChooser.addObject("Scale", new RightScaleFromP1());
			rrChooser.addObject("Scale", new RightScaleFromP1());
			
			rlChooser.addObject("Scale", new LeftScaleFrontFromP1());

			llChooser.addObject("Scale and Switch", new LeftScaleAndSwitchFromP1());
			
			llChooser.addObject("Left Scale Side", new LeftScaleSideFromP1());
			rlChooser.addObject("Left Scale Side", new LeftScaleSideFromP1());
			break;
		case 3:
			llChooser.addObject("Center to Left Switch", new LeftSwitchFromP3());
			lrChooser.addObject("Center to Left Switch", new LeftSwitchFromP3());
			rlChooser.addObject("Center to Right Switch", new RightSwitchFromP3());
			rrChooser.addObject("Center to Right Switch", new RightSwitchFromP3());
			break;
		case 4:
			rlChooser.addObject("Switch Front", new StraightToSwitch());
			rrChooser.addObject("Switch Front", new StraightToSwitch());
			break;
		case 5:
			rlChooser.addObject("Switch Side", new SideOfRightSwitchFromP5());
			rrChooser.addObject("Switch Side", new SideOfRightSwitchFromP5());

			llChooser.addObject("Scale", new LeftScaleFromP5());
			rlChooser.addObject("Scale", new LeftScaleFromP5());
			
			lrChooser.addObject("Scale", new RightScaleFrontFromP5());
			
			rrChooser.addObject("Scale and Switch", new RightScaleAndSwitchFromP5());
			
			rrChooser.addObject("Right Scale Side", new RightScaleSideFromP5());
			lrChooser.addObject("Right Scale Side", new RightScaleSideFromP5());
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
