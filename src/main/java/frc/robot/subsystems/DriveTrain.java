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

  static WPI_TalonSRX talonLeft;
  static WPI_TalonSRX talonRight;
  static WPI_VictorSPX victorLeft;
  static WPI_VictorSPX victorRight;
  AHRS ahrs;

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

  /**
   *  TODO: fill out comments
   */
  public void writeToSmartDashboard(){
    SmartDashboard.putNumber("Left Encoder Distance", talonLeft.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder Distance", talonRight.getSelectedSensorPosition(0));
  }
}
