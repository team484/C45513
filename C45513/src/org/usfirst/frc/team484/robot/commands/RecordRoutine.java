package org.usfirst.frc.team484.robot.commands;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.RobotState;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Records the robot state each cycle and saves the information into a 
 * serialized array. The array is saved to non-volitile storage for later use.
 * Information included in the robot state includes position, power output, and
 * speed of various components.
 */
public class RecordRoutine extends Command {
	private static final String SAVE_DIR = RobotSettings.SAVE_DIR_ROUTINES;
	FileOutputStream fos;
	ObjectOutputStream oos;
	ArrayList<RobotState> stateArray = new ArrayList<>();
	String name;

	/**
	 * Creates a new RecordRoutine command which will write the robot state to
	 * a file with the name given.
	 * @param routineName - Name of the robot routine file.
	 */
	public RecordRoutine(String routineName) {
		name = routineName;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (!new File(SAVE_DIR + name).exists()) {
			new File(SAVE_DIR + name).mkdirs();
		}
		try {
			fos = new FileOutputStream(SAVE_DIR + "/" + name + ".rtn");
			oos = new ObjectOutputStream(fos);
		} catch (Exception e) {
			e.printStackTrace();
		}
		RobotIO.setVoltageComp(true);
		RobotIO.leftEncoder.reset();
		RobotIO.rightEncoder.reset();
		RobotIO.imu.setYaw(0, 100);
		stateArray.clear();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		stateArray.add(new RobotState());
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		try {
			oos.writeObject(stateArray);
			oos.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		RobotIO.setVoltageComp(false);
	}
}
