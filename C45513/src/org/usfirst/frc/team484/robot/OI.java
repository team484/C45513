/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import org.usfirst.frc.team484.robot.commands.ClimbWithJoystick;
import org.usfirst.frc.team484.robot.commands.ClimberDoNothing;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveDistance;
import org.usfirst.frc.team484.robot.commands.GrabberAngleDoNothing;
import org.usfirst.frc.team484.robot.commands.JoystickDrive;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;
import org.usfirst.frc.team484.robot.commands.ToggleGrabber;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Creates all joystick button objects and assigns them to commands
 */
public class OI {
	private Button shiftUp;
	private Button shiftDown;
	private Button lowerGrabber;
	private Button raiseGrabber;
	private Button  toggleGrabber;
	private Button climbButton;
	
	private Button test;


	public OI() {
		
		//-----Initialize button on driver stick-----
		try {
			shiftUp = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_UP_BUTTON);
			shiftDown = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_DOWN_BUTTON);
			test = new JoystickButton(RobotIO.driveStick, 8);
			test.whenPressed(new DriveDistance(250));
			test.whenReleased(new JoystickDrive());
			
			shiftUp.whenPressed(new ShiftUp());
			shiftDown.whenPressed(new ShiftDown());
		} catch (Throwable t) {
			t.printStackTrace();
		}
		
		//-----Initialize buttons on operator stick-----
		try {
			lowerGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.LOWER_GRABBER_BUTTON);
			raiseGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.RAISE_GRABBER_BUTTON);
			toggleGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.TOGGLE_GRABBER_BUTTON);
			climbButton = new JoystickButton(RobotIO.opStick, RobotSettings.CLIMBER_BUTTON);

			lowerGrabber.whenPressed(new RotateGrabberDown(RobotSettings.GRABBER_ROTATE_SPEED_DOWN));
			raiseGrabber.whenPressed(new RotateGrabberUp(RobotSettings.GRABBER_ROTATE_SPEED_UP));
			lowerGrabber.whenReleased(new GrabberAngleDoNothing());
			raiseGrabber.whenReleased(new GrabberAngleDoNothing());
			toggleGrabber.whenPressed(new ToggleGrabber());
			climbButton.whileHeld(new ClimbWithJoystick());
			climbButton.whenReleased(new ClimberDoNothing());
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}
}
