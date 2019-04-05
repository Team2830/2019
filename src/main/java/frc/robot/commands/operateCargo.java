/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    XboxController operatorController = Robot.oi.getOperatorController();

    if(operatorController.getBButton()){
      Robot.cargo.cargoIntakeIn();
    } else if(operatorController.getAButton()){
      Robot.cargo.cargoIntakeOut();
    } else{
      Robot.cargo.stopCargoIntake();
    }

    if(operatorController.getTriggerAxis(GenericHID.Hand.kRight) > .3){
      Robot.cargo.cargoUp();
    }

    if(operatorController.getTriggerAxis(GenericHID.Hand.kLeft) > .3){
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
