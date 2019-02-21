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

public class operateClimber extends Command {
  public operateClimber() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick operatorStick = Robot.oi.getOperatorJoystick();
    Joystick driverStick = Robot.oi.getDriverJoystick();



    if(operatorStick.getRawButton(7)){
      if(Math.abs(driverStick.getRawAxis(5)) > 0.2){
        Robot.climber.driveClimber(driverStick);
      } else{
        Robot.climber.stopClimber();
      }

      if(driverStick.getRawButton(3)){
        Robot.climber.clamp();
      }

      if(driverStick.getRawButton(4)){
        Robot.climber.unClamp();
      }
    } else{
      Robot.climber.stopClimber();
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
