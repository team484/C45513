package org.usfirst.frc.team484.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;

/**
 * This class keeps track of the robot's location and orientation (pose) and 
 * records this state every 1ms. When end() is called, the trajectory created
 * by these poses will be saved to a file.
 */
public class PoseTracker {
	private PigeonIMU imu;
	private Encoder leftEnc, rightEnc;
	private static final double ENC_DIFF_PER_DEGREE = RobotSettings.ENC_DIFF_PER_DEGREE;
	private static final String SAVE_DIR = RobotSettings.SAVE_DIR_POSES;
	
	/**
	 * Create a new pose tracker
	 * @param imu - the Pigeon IMU being used to keep track of robot rotation
	 * @param leftEnc - the encoder for the left side of the robot
	 * @param rightEnc - the encoder for the right side of the robot
	 */
	public PoseTracker(PigeonIMU imu, Encoder leftEnc, Encoder rightEnc) {
		this.imu = imu;
		this.leftEnc = leftEnc;
		this.rightEnc = rightEnc;
	}
	
	private double imuAngleOffset = 0;
	private double encoderAngleOffset;
	private ArrayList<Pose> encoderPoses = new ArrayList<>();
	private ArrayList<Pose> gyroPoses = new ArrayList<>();
	
	public Pose getLastGyro() {
		if (gyroPoses.size() < 1) {
			return new Pose(0, 0, 0, 0);
		}
		return gyroPoses.get(gyroPoses.size() - 1);
	}
	
	public Pose getLastEncoder() {
		if (encoderPoses.size() < 1) {
			return new Pose(0, 0, 0, 0);
		}
		return encoderPoses.get(encoderPoses.size() - 1);
	}
	
	/**
	 * Start tracking robot pose over time
	 * @param startX - the robot's current x coordinate
	 * @param startY - the robot's current y coordinate
	 * @param startAngle - the angle the robot is currently facing
	 */
	public void begin(double startX, double startY, double startAngle) {
		if (trackingThread != null && trackingThread.isAlive()) {
			return;
		}
		Pose startPose = new Pose(startX, startY, startAngle,0);
		encoderPoses.add(startPose);
		gyroPoses.add(startPose);
		imuAngleOffset = startAngle - getIMUAngle();
		encoderAngleOffset = startAngle;
		trackingThread = new Thread() {
			public void run() {
				//Loop through adding poses until the thread is told to stop
				while(!isInterrupted()) {
					//Calculate angle of robot using difference in encoder values
					double encoderDifference = rightEnc.getDistance() - leftEnc.getDistance();
					double encoderAngleDelta = encoderDifference * ENC_DIFF_PER_DEGREE;
					double angleEncoder = encoderAngleDelta + encoderAngleOffset;
					//Calculate angle of robot using IMU readings
					double angleGyro = getIMUAngle() + imuAngleOffset;
					//Distance robot has traveled along trajectory
					double distance = (leftEnc.getDistance() + rightEnc.getDistance()) / 2.0;
					
					//Calculate robot pose using encoders
					Pose lastEncoderPose = encoderPoses.get(encoderPoses.size() - 1);
					double deltaDistance = distance - lastEncoderPose.distance;
					double deltaX = Math.cos(Math.toRadians(angleEncoder)) * deltaDistance;
					double deltaY = Math.sin(Math.toRadians(angleEncoder)) * deltaDistance;
					Pose newEncoderPose = new Pose(lastEncoderPose.x + deltaX, lastEncoderPose.y + deltaY, angleEncoder, distance);
					encoderPoses.add(newEncoderPose);
					
					//Calculate robot pose using IMU gyro
					Pose lastGyroPose = gyroPoses.get(gyroPoses.size() - 1);
					deltaDistance = distance - lastGyroPose.distance;
					deltaX = Math.cos(angleGyro) * deltaDistance;
					deltaY = Math.sin(angleGyro) * deltaDistance;
					Pose newGyroPose = new Pose(lastGyroPose.x + deltaX, lastGyroPose.y + deltaY, angleGyro, distance);
					gyroPoses.add(newGyroPose);
					
					//Take a break before adding a new pose to the list
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {
						break;
					}
					
				}
				//Create save directory if it does not exist
				if (!new File(SAVE_DIR).exists()) {
					new File(SAVE_DIR).mkdirs();
				}
				//Gets timestamp to add to save file name
				String timeStamp = new SimpleDateFormat("MM.dd.HH.mm.ss").format(new Date());
				try {
					//Creates a BufferedWriter object to write to new save file
					BufferedWriter poseWriter = new BufferedWriter(new FileWriter(SAVE_DIR + timeStamp + "_poses.csv"));
					//First line of CSV file shows the name for each column
					poseWriter.write("gyro_x,gyro_y,gyro_angle,encoder_x,encoder_y,encoder_angle");
					//Loop through each pose and write the state to the CSV
					for (int i = 0; i < encoderPoses.size() && i < gyroPoses.size(); i++) {
						String line;
						if (gyroPoses.size() > i) {
							line = gyroPoses.get(i).x * 0.0254 + "," + gyroPoses.get(i).y * 0.0254 + "," + gyroPoses.get(i).angle + ",";
						} else {
							line = ",";
						}
						/*if (encoderPoses.size() > i) {
							line = encoderPoses.get(i).x + "," + encoderPoses.get(i).y + "," + encoderPoses.get(i).angle;
						} else {
							line = ",,";
						}*/
						poseWriter.write(line);
						poseWriter.newLine();
					}
					poseWriter.close();
				} catch (IOException e) {
					e.printStackTrace();
				}

			}
		};
		trackingThread.start();
	}
	
	/**
	 * End tracking of robot pose and save robot trajectory to a file.
	 */
	public void end() {
		if (trackingThread == null) {
			return;
		}
		if (trackingThread.isAlive()) {
			trackingThread.interrupt();
		}
	}
	
	/*
	 * This thread runs when tracking the robot pose and will do so until it is
	 * interrupted.
	 */
	Thread trackingThread;
	
	/**
	 * Gets the delta angle provided by the robot IMU
	 * @return Relative angle since robot was turned on
	 */
	private double getIMUAngle() {
		double[] ypr = new double[3];
		imu.getYawPitchRoll(ypr);
		return ypr[0];
	}
	
	/**
	 * Struct to store robot pose at a particular moment in time
	 *
	 */
	public class Pose {
		public double x,y,angle,distance;
		/**
		 * 
		 * @param x - x coordinate of robot
		 * @param y - y coordinate of robot
		 * @param angle - angle of robot
		 * @param distance - distance robot has traveled
		 */
		public Pose(double x, double y, double angle, double distance) {
			this.x = x;
			this.y = y;
			this.angle = angle;
			this.distance = distance;
		}
	}
}
