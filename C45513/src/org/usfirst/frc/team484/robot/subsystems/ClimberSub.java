package org.usfirst.frc.team484.robot.subsystems;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.commands.ClimberDoNothing;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for controlling the climber motors
 */
public class ClimberSub extends Subsystem {

	/**
	 * By default, we want the climber to do nothing. This sets the default
	 * command to ClimberDoNothing.
	 */
    public void initDefaultCommand() {
        setDefaultCommand(new ClimberDoNothing());
    }
    
    /**
     * Sets the speed for the climber motors. Positive is up.
     * @param speed - climbing speed [-1.0 to 1.0]
     */
    public static void climb(double speed) {
    		if (RobotIO.leftClimberMotor == null) return;
    		if (RobotIO.rightClimberMotor == null) return;
    		RobotIO.leftClimberMotor.set(speed);
    		RobotIO.rightClimberMotor.set(speed);
    }
}

