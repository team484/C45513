/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import org.usfirst.frc.team484.robot.commands.GrabberAngleDoNothing;
import org.usfirst.frc.team484.robot.commands.JoystickDrive;
import org.usfirst.frc.team484.robot.commands.JoystickElevator;
import org.usfirst.frc.team484.robot.commands.PIDElevateDownToHeight;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.RotateToCube;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;
import org.usfirst.frc.team484.robot.commands.TeleScaleCube;
import org.usfirst.frc.team484.robot.commands.ToggleGrabber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Creates all joystick button objects and assigns them to commands
 */
public class OI {
	private static Button shiftUp;
	private static Button shiftDown;
	private static Button triggerShifting;
	private static Button lowerGrabber;
	private static Button raiseGrabber;
	private static Button toggleGrabber;
	private static Button teleCubeButton;
	
	private static Button toggleElevatorUp;
	private static Button toggleElevatorDown;

	private static Button test;
	
	private static boolean isDriveStickSetup = false;
	private static boolean isOpStickSetup = false;

	public static void setupOI() {

		//-----Initialize button on driver stick-----
		try {
			if ((!RobotIO.driveStick.getName().equals("")
				|| DriverStation.getInstance().getMatchType() != MatchType.None)
					&& !isDriveStickSetup) {
				shiftUp = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_UP_BUTTON);
				shiftDown = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_DOWN_BUTTON);
				triggerShifting = new JoystickButton(RobotIO.driveStick, 1);
				test = new JoystickButton(RobotIO.driveStick, 8);
				teleCubeButton = new JoystickButton(RobotIO.driveStick, 4);
				test.whenPressed(new PIDElevateDownToHeight(0.0));
				test.whenReleased(new JoystickElevator());

				shiftUp.whenPressed(new ShiftUp());
				shiftDown.whenPressed(new ShiftDown());
				triggerShifting.whenPressed(new ShiftUp());
				triggerShifting.whenReleased(new ShiftDown());
				
				teleCubeButton.whenPressed(new TeleScaleCube());
				teleCubeButton.whenReleased(new JoystickDrive());
				teleCubeButton.whenReleased(new JoystickElevator());
				
				isDriveStickSetup = true;
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}

		//-----Initialize buttons on operator stick-----
		try {
			if ((!RobotIO.driveStick.getName().equals("")
					|| DriverStation.getInstance().getMatchType() != MatchType.None)
						&& !isOpStickSetup) {
				lowerGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.LOWER_GRABBER_BUTTON);
				raiseGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.RAISE_GRABBER_BUTTON);
				toggleGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.TOGGLE_GRABBER_BUTTON);
				toggleElevatorUp = new JoystickButton(RobotIO.opStick, 4);
				toggleElevatorDown = new JoystickButton(RobotIO.opStick, 5);
				
				lowerGrabber.whenPressed(new RotateGrabberDown(RobotSettings.GRABBER_ROTATE_SPEED_DOWN));
				raiseGrabber.whenPressed(new RotateGrabberUp(RobotSettings.GRABBER_ROTATE_SPEED_UP));
				lowerGrabber.whenReleased(new GrabberAngleDoNothing());
				raiseGrabber.whenReleased(new GrabberAngleDoNothing());
				toggleGrabber.whenPressed(new ToggleGrabber());
				toggleElevatorUp.whileHeld(new PIDElevateUpToHeight(1.0));
				toggleElevatorUp.whenReleased(new JoystickElevator());
				toggleElevatorDown.whileHeld(new PIDElevateDownToHeight(0.0));
				toggleElevatorDown.whenReleased(new JoystickElevator());
				isOpStickSetup = true;
			}
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}
}
