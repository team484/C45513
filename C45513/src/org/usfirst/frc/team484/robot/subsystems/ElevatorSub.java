package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.JoystickElevator;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for controlling the cube elevator.
 */
public class ElevatorSub extends Subsystem {

	/**
	 * Sets the command that runs for this subsystem when no
	 * other commands are running that require the subsystem.
	 */
    public void initDefaultCommand() {
        setDefaultCommand(new JoystickElevator());
    }
    
    /**
     * Applies an output of 0 to the elevator motor.
     */
    public static void doNothing() {
		if (RobotIO.elevatorMotor == null) return;
    		RobotIO.elevatorMotor.set(0);
    }
    
    /**
     * Sets the output power of the elevator motor.
     * @param speed - the power output from -1 to 1.
     */
    public static void setRate(double speed) {
    		if (RobotIO.elevatorMotor == null) return;
    		RobotIO.elevatorMotor.set(speed - RobotSettings.ELEVATOR_GRAVITY_COMPENSATION_POWER);
    }
}

