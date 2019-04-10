/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 *
 */
public class FollowTrajectory extends Command {

	EncoderFollower left, right;
	File leftTrajectoryPathFile;
	File rightTrajectoryPathFile;
  Notifier periodicRunnable;
  Trajectory leftTrajectory;
  Trajectory rightTrajectory;
	double positionDifferenceLeft = 0;
	double positionDifferenceRight = 0;
	double angleDifference = 0;
	NetworkTableEntry leftSideEntry, rightSideEntry, angleEntry;

  public FollowTrajectory() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
	this("/home/lvuser/deploy/rightcargo1_left.csv", "/home/lvuser/deploy/rightcargo1_right.csv");
//	Robot.driveTrain.m_drive.close();
}

  public FollowTrajectory(String leftPathFile, String rightPathFile) {
      // Use requires() here to declare subsystem dependencies
      // eg. requires(chassis);
    requires(Robot.driveTrain);
	leftTrajectoryPathFile = new File(leftPathFile);
	rightTrajectoryPathFile = new File(rightPathFile);

	
    ShuffleboardLayout followTrajectoryMappingList = Shuffleboard.getTab("Motion")
    .getLayout("Left", BuiltInLayouts.kList)
    .withSize(10,4)
    .withPosition(0,0)
    .withProperties(Map.of("Label position", "LEFT"));

    leftSideEntry = followTrajectoryMappingList.add("Left Position Difference", positionDifferenceLeft).withWidget(BuiltInWidgets.kGraph).withPosition(0,0).getEntry();
	rightSideEntry = followTrajectoryMappingList.add("Right Position Difference", positionDifferenceRight).withWidget(BuiltInWidgets.kGraph).withPosition(0,1).getEntry();
	angleEntry = followTrajectoryMappingList.add("Angle Difference", angleDifference).withWidget(BuiltInWidgets.kGraph).withPosition(0,2).getEntry();
}
  // Called just before this Command runs the first time
  protected void initialize() {
    Robot.driveTrain.resetCounters();
    Robot.driveTrain.writeToSmartDashboard();
/*
* 		We aren't using waypoints here to generate the path dynamically, but if we did, this is how you would do it.
  Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(10, 0, 0)
  };
    	
		 Arguments:
		 Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
		 Sample Count:        SAMPLES_HIGH (100 000)
		                      SAMPLES_LOW  (10 000)
		                      SAMPLES_FAST (1 000)
		 Time Step:           0.02 Seconds
		 Max Velocity:        10.36 f/s
		 Max Acceleration:    2.0 f/s/s
		 Max Jerk:            60.0 f/s/s/s
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 5, 2.0, 60.0);
		Trajectory trajectory = Pathfinder.generate(points, config); */

		try {
			leftTrajectory = Pathfinder.readFromCSV(leftTrajectoryPathFile);
			rightTrajectory = Pathfinder.readFromCSV(rightTrajectoryPathFile);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		// Setting wheel base distance/
		try {
            left.reset();
            right.reset();
        } catch (Exception e) {
        }

		left = new EncoderFollower(leftTrajectory);
    	right = new EncoderFollower(rightTrajectory);
    	
/*    	 Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the 
    	 'getEncPosition' function.
    	 1000 is the amount of encoder ticks per full revolution
    	 Wheel Diameter is the diameter of your wheels (or pulley for a track system) in feet*/

    	left.configureEncoder(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER), 4096, .5);
    	right.configureEncoder(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER), 4096, .5);
    	
/*    	 The first argument is the proportional gain. Usually this will be quite high
    	 The second argument is the integral gain. This is unused for motion profiling
    	 The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    	 The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    	      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    	 The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker*/
    	//left.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .2);
    	//right.configurePIDVA(3.6, 0.0, 0.2, 1/10.36, .2);
		
		left.configurePIDVA(3.6, 0.0, 0, 1/6, .1);
    	right.configurePIDVA(3.6, 0.0, 0, 1/6, .1);
    	

        // Initialize the Notifier
        periodicRunnable = new Notifier(new PeriodicRunnable());

        // Start the notifier (notifier will be called every 0.02 second to ensure trajectory is read properly)
        periodicRunnable.startPeriodic(0.02);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(left.isFinished() && right.isFinished()){
        	System.out.println("finished reach called");
    		return true;

    	}return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stopDriving();
    	periodicRunnable.stop();
    	System.out.println("end called");
    	Robot.driveTrain.writeToSmartDashboard();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveTrain.stopDriving();
    	periodicRunnable.stop();    	
    }
    class PeriodicRunnable implements Runnable {
        int segment = 0;
    	int counter = 0;

        @Override
        public void run() {
        	double l = left.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
        	double r = right.calculate(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));

		//	leftSideEntry.setDouble(left.getSegment().position -  (Robot.driveTrain.getInchesFromPulses(Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER))/12));
	//		rightSideEntry.setDouble(right.getSegment().position - (Robot.driveTrain.getInchesFromPulses(Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER))/12));
		

        	double gyro_heading = Robot.driveTrain.getAngle();    // Assuming the gyro is giving a value in degrees
        	double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			double turn =  0.8 * (-1.0/80.0) * angleDifference;
		//	angleEntry.setDouble(angleDifference);

        	Robot.driveTrain.setLeft(l + turn);
        	Robot.driveTrain.setRight(r - turn);
        	
//        	if (counter % 10 == 0 && !left.isFinished() && !right.isFinished()){
        	//	System.out.printf("Left Vel: %f\t", left.getSegment().velocity); 
			//	System.out.printf("Left Acc: %f\t", left.getSegment().acceleration);
			// System.out.printf("gyro: %f\t", Robot.driveTrain.getAngle());
			// 	System.out.printf("Left Pos: %f\t", left.getSegment().position);
			// 	System.out.printf("L: %f\t", l);
			// 	System.out.printf ("left enc: %d\t",Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
			// 	System.out.printf("Right Pos: %f\t", right.getSegment().position);
			// 	System.out.printf("R: %f\t", r);
        	// 	System.out.printf("right enc: %d\t",Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));
			// 	System.out.printf("Left Vel: %f\t", left.getSegment().velocity);
			// 	System.out.printf("Right Vel: %f\t", right.getSegment().velocity);
			// 	System.out.printf("Left Acc: %f\t", left.getSegment().acceleration);
			// 	System.out.printf("Right Acc: %f\t", right.getSegment().acceleration);

     //   		System.out.printf("Left Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.LEFT_ENCODER));
     //   		System.out.printf("Left Output: %f\t", l);
//        		System.out.printf("R-Y: %f\t", right.getSegment().y );
//        		System.out.printf("L-Y: %f\t", left.getSegment().y );
//        		System.out.printf("R-X: %f\t", right.getSegment().x );
//        		System.out.printf("L-X: %f\t", left.getSegment().x );

        		//	System.out.printf("Right Encoder: %d\t", Robot.driveTrain.getEncoderValue(DriveTrain.RIGHT_ENCODER));
        	//	System.out.printf("Right Output: %f\t", r);
        	//	System.out.printf("Angle Diff: %f\t",angleDifference);
        	//	System.out.printf("angle correction: %f\n", turn);
  //      		System.out.println("end");
    //    	}
      //  	counter++;
        //	Robot.driveTrain.writeToSmartDashboard();
        }
    }
}