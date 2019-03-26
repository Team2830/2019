/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberEnabled extends Command {
  public ClimberEnabled() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick operatorJoystick = Robot.oi.getOperatorJoystick();
    Joystick driverStick = Robot.oi.getDriverJoystick();
    if(operatorJoystick.getRawButton(7)){
      if(driverStick.getRawButton(1)){
        Robot.climber.backDown();
      } else if(driverStick.getRawButton(2)){
        Robot.climber.backUp();}
        else if(driverStick.getRawButton(3)){
          Robot.climber.frontDown();}
          else if(driverStick.getRawButton(4)){
            Robot.climber.frontUp();
          }
        
      }
    }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
