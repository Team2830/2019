/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * @param m_distance
 *            is the goal distance of the robot
 * @param m_speed
 *            is the speed the robot will drive
 */
public class DriveDistance extends Command {
	private double m_distance;
	private double m_speed;
	private double delay;
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;
	
	private boolean isFirstRun = true;
	
	public DriveDistance(double distance, double speed, double delaySeconds) {
		System.out.println("drive: start");
		
		m_distance = Robot.driveTrain.getPulsesFromInches(distance);
		m_speed = speed;
		delay = delaySeconds;
		
		requires(Robot.driveTrain);
	}

	public DriveDistance(double distance) {
		this(distance, 0.7, 0);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("drive: init: "+m_distance+", "+m_speed+", "+delay);
		SmartDashboard.putString("Current Command", "Drive Forward");
		Robot.driveTrain.resetCounters();
		Robot.driveTrain.setOpenloopRamp(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
    	if(isFirstRun){
    		Robot.driveTrain.resetCounters();
    		System.out.println("drive: execute");
    		isFirstRun = false;
    	}
    	SmartDashboard.putNumber("Encoder Distance Drive", Robot.driveTrain.getDistance());
    	/*if(Robot.drivetrain.getAnalogGyro1().getAngle()>1.00){
    		Robot.drivetrain.driveForward(m_speed,.3);
    	}
    	else if(Robot.drivetrain.getAnalogGyro1().getAngle()<-1.00){
    		Robot.drivetrain.driveForward(m_speed,-.3);
    	}
    	else{
    		Robot.drivetrain.driveForward(m_speed,0);
    	}*/
        double v;
       	double x = Robot.driveTrain.getDistance();
       	double rampDown = Robot.driveTrain.getPulsesFromInches(Math.copySign(70, m_distance));
       	double rampUp = Robot.driveTrain.getPulsesFromInches(Math.copySign(18, m_distance));
       	double vMin = Math.copySign(.2, m_distance);
       	double xRamp = Math.min(rampUp,m_distance/2);
       	double xBrake = m_distance-Math.min(rampDown,m_distance/2);
       	if (Math.abs(x)>Math.abs(xBrake)){
//       		v=(vMin-m_speed)/(m_distance-xBrake)*(2*(x-m_distance));
       		v= m_speed-((m_speed-vMin)/(m_distance-xBrake))*(x-xBrake);
       	}
       	else if (Math.abs(x)<Math.abs(xRamp))
       		v=vMin+((m_speed-vMin)/xRamp)*x;
       	else
      		v=m_speed;
       	if(m_distance <= 15){
       		v = .5;
       	}
       	SmartDashboard.putNumber("Ramp Velocity", v);
       	SmartDashboard.putNumber("Distance Driven", Robot.driveTrain.getInchesFromPulses(x));
       	
       	
    	Robot.driveTrain.driveStraight(v);
       	
    }
	
	private boolean detectCollision(){

		final double kCollisionThreshold_DeltaG = 0.5f;
		
		boolean collisionDetected = false;
        
        double curr_world_linear_accel_x = Robot.driveTrain.getAccelerationX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = Robot.driveTrain.getAccelerationY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        
        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
             ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
            collisionDetected = true;
        }
        SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);
        return collisionDetected;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (m_distance <= Robot.driveTrain.getDistance()|| this.isTimedOut()) {
			Robot.driveTrain.driveStraight(0);
			Timer.delay(delay);
			Robot.driveTrain.resetCounters();
			System.out.println("drive: finished: " +Robot.driveTrain.getDistance());
			return true;
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("drive: end");
		Robot.driveTrain.driveStraight(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("drive: interrupted");
		Robot.driveTrain.driveStraight(0);
	}
}

