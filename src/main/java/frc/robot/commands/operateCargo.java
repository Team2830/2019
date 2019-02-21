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

public class operateCargo extends Command {
  public operateCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.cargo);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick operatorStick = Robot.oi.getOperatorJoystick();

    if(operatorStick.getRawButton(1)){
      Robot.cargo.cargoIntakeIn();
    } else if(operatorStick.getRawButton(2)){
      Robot.cargo.cargoIntakeOut();
    } else{
      Robot.cargo.stopCargoIntake();
    }

    if(operatorStick.getRawAxis(2) > .3){
      Robot.cargo.cargoUp();
    }

    if(operatorStick.getRawAxis(3) > .3){
      Robot.cargo.cargoDown();
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