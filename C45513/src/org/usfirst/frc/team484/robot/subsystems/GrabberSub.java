package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem that controls the state of the cube grabber.
 */
public class GrabberSub extends Subsystem {

	/**
	 * There is no default command for this subsystem
	 */
	public void initDefaultCommand() { }

	/**
	 * Closes the cube grabber.
	 */
	public static void closeGrabber() {
		if (RobotIO.grabberSolenoid == null) return;
		RobotIO.grabberSolenoid.set(Value.kForward);
	}
	
	/**
	 * Opens the cube grabber.
	 */
	public static void openGrabber() {
		if (RobotIO.grabberSolenoid == null) return;
		RobotIO.grabberSolenoid.set(Value.kReverse);
	}
	
	/**
	 * Toggles the grabber between open and close states
	 */
	public static void toggleGrabber() {
		if (RobotIO.grabberSolenoid == null) return;
		if (RobotIO.grabberSolenoid.get() == Value.kForward) {
			openGrabber();
		} else {
			closeGrabber();
		}
	}
}

