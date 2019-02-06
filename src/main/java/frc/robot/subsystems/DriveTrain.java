/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  public static WPI_TalonSRX talonLeft;
  public static WPI_TalonSRX talonRight;
  static WPI_VictorSPX victorLeft;
  static WPI_VictorSPX victorRight;
  static AHRS ahrs;
  int collisionCount = 0;
  boolean collisionDetected = false;

  public static int MotionMagicLoop = 0;
  public static int TimesInMotionMagic = 0;

  final static double kCollisionThreshold_Delta6 = 2f;

  int kPIDLoopIdx = 0;
  int kSlotIdx = 0;
  int kTimeoutMs = 30;
  double kP = 0.2;
  double kI = 0.0;
  double kD = 0.0;
  double peakOutput = 1.0;

public DriveTrain() {
  talonLeft = new WPI_TalonSRX(15);
  talonRight = new WPI_TalonSRX(20);
  victorLeft = new WPI_VictorSPX(14);
  victorRight = new WPI_VictorSPX(21);
  ahrs = new AHRS(SPI.Port.kMXP);

  talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

  talonRight.setInverted(true);
  victorRight.setInverted(true);
  talonLeft.setInverted(false);
  victorLeft.setInverted(false);

  victorLeft.follow(talonLeft);
  victorRight.follow(talonRight);

  
}
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

public double joystickDeadband =0.04;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new DriveCommand());
  }


  public void arcadeDrive(Joystick joystick){

    double[] values = drive(joystick);
    double direction = values[0], leftSpeed = values[1], rightSpeed = values[2];
    if(direction == 0 || direction == 3){
      talonLeft.set(rightSpeed);
      talonRight.set(leftSpeed);
    } else{
      talonRight.set(rightSpeed);
      talonLeft.set(leftSpeed);
    }
  }
/**
 * 
 * @param joystick
 * @return double[] 3 values, the direction, the leftSpeed and the right speed
 */
  public double[] drive(Joystick joystick){
    double throttle = deadbanded((-1*joystick.getRawAxis(2))+joystick.getRawAxis(3), joystickDeadband);
    if (Math.abs(throttle) > 1){
      throttle = Math.copySign(1, throttle);
    }
    double steering = 0.6*deadbanded(joystick.getRawAxis(0), joystickDeadband); 
    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(steering)), throttle);
    double speed, direction;
    if (throttle >= 0){
      if(steering >= 0){
        direction = 0;
        speed = throttle-steering;
      }else{
        direction = 1;
        speed = throttle+steering;
      }

    }else{
      if(steering >= 0){
        direction = 2;
        speed = throttle+steering;
      }else{
        direction = 3;
        speed = throttle-steering;
      }
    }
    
    double[] set = {direction,speed,maxInput};
    DetectCollision();
    if (collisionDetected == true){
      if (collisionCount < 5){
        speed = 0;
        maxInput = 0;
        collisionCount++;
      }
      else {
        collisionDetected = false;
        collisionCount = 0;
      }
    }
    return set;
  }

  /**
   *  TODO: fill out comments
   * 
   * @param joystick
   */
  public void visionDrive(Joystick joystick){
    double tx = Robot.vision.getHorizontalOffset();
    double Kp = -0.05;
    double steering_adjust = 0.0;
    if(joystick.getRawButton(4) && Robot.vision.hasTargets()) {
      SmartDashboard.putBoolean("buttonpressed",true);   
      if(tx > 1.0){
        steering_adjust = Kp*(-tx);
      }
      else if(tx < 1.0){
        steering_adjust = Kp*-tx;
      }
    }
    SmartDashboard.putNumber("steeringAdjust",steering_adjust);
    Robot.vision.writeToSmartDashboard();
    double[] values = drive(joystick);
    double direction = values[0], leftSpeed = values[1], rightSpeed = values[2];
    if(direction == 0){
      talonLeft.set(rightSpeed);
      talonRight.set(leftSpeed-steering_adjust);
    } else if(values[0] == 1){
      talonRight.set(rightSpeed);
      talonLeft.set(leftSpeed+steering_adjust);
    } else if(values[0] == 2){
      talonRight.set(rightSpeed);
      talonLeft.set(leftSpeed-steering_adjust);
    } else{
      talonRight.set(rightSpeed);
      talonLeft.set(leftSpeed+steering_adjust);
    }
  }
  public void resetCounters(){
    ahrs.zeroYaw();
    talonLeft.setSelectedSensorPosition(0, 0, 10);
    talonRight.setSelectedSensorPosition(0, 0, 10);
  }
  public void driveForward(){
  talonRight.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
  talonLeft.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
  talonLeft.config_kP(kSlotIdx, kP, kTimeoutMs);
  talonRight.config_kP(kSlotIdx, kP, kTimeoutMs);
  talonLeft.config_kI(kSlotIdx, kI, kTimeoutMs);
  talonRight.config_kI(kSlotIdx, kI, kTimeoutMs);
  talonLeft.config_kD(kSlotIdx, kD, kTimeoutMs);
  talonRight.config_kD(kSlotIdx, kD, kTimeoutMs);
  double leftMotorOutput = talonLeft.getMotorOutputPercent();
  double rightMotorOutput = talonRight.getMotorOutputPercent();
  }

  /**
   * 
   *  TODO: fill out comments
   * 
   * @param input
   * @param deadband
   * @return
   */
    public double deadbanded(double input, double deadband){
      if(Math.abs(input)>Math.abs(deadband)){
        return input;
      }else{
        return 0;
      }
  }
  
  public void DetectCollision(){

    double lastWorldLinearAccelX = 0;
    double lastWorldLinearAccelY = 0;

      double currWorldLinearAccelX = ahrs.getWorldLinearAccelX();
      double currentJerkX = currWorldLinearAccelX - lastWorldLinearAccelX;
      lastWorldLinearAccelX = currWorldLinearAccelX;
      double currWorldLinearAccelY = ahrs.getWorldLinearAccelY();
      double currentJerkY = currWorldLinearAccelY - lastWorldLinearAccelY;
      lastWorldLinearAccelY = currWorldLinearAccelY;

      if ((Math.abs(currentJerkX) > kCollisionThreshold_Delta6) ||
        (Math.abs(currentJerkY) > kCollisionThreshold_Delta6)) {
          collisionDetected = true;
        }
  }
    
  public int getPulsesFromInches(double inches){
		return (int)(240/Math.PI*inches);
	}

	public double getInchesFromPulses(double d){
		return d*Math.PI/240;
  }
  
  public double getAngle(){
    return ahrs.getAngle();
  }

  public double getDistance(){
		return(talonLeft.getSelectedSensorPosition(0) + talonRight.getSelectedSensorPosition(0))/2;
	}
  

  /**
   *  TODO: fill out comments
   */
  public void writeToSmartDashboard(){
    SmartDashboard.putNumber("Left Encoder Distance", talonLeft.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder Distance", talonRight.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
    SmartDashboard.putBoolean("Collision Detected", collisionDetected);
    SmartDashboard.putNumber("Sensor Velocity", talonRight.getSelectedSensorPosition(kPIDLoopIdx));
    SmartDashboard.putNumber("Sensor Position", talonRight.getActiveTrajectoryVelocity());
    SmartDashboard.putNumber("Motor Output Precent", talonRight.getMotorOutputPercent());
    SmartDashboard.putNumber("Closed Loop Error", talonRight.getClosedLoopError(kPIDLoopIdx));
  }
}
