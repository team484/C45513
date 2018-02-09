/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team484.robot;

import org.usfirst.frc.team484.robot.commands.CloseGrabber;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Creates all joystick button objects and assigns them to commands
 */
public class OI {
	private JoystickButton shiftUp = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_UP_BUTTON);
	private JoystickButton shiftDown = new JoystickButton(RobotIO.driveStick, RobotSettings.SHIFT_DOWN_BUTTON);
	private JoystickButton closeGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.CLOSE_GRABBER_BUTTON);
	private JoystickButton openGrabber = new JoystickButton(RobotIO.opStick, RobotSettings.OPEN_GRABBER_BUTTON);

	public OI() {
			shiftUp.whenPressed(new ShiftUp());
			shiftDown.whenPressed(new ShiftDown());
			closeGrabber.whenPressed(new CloseGrabber());
			openGrabber.whenPressed(new OpenGrabber());
	}
}
