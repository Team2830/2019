/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;

public class JoystickRumble extends Command {
  XboxController joystick;
  int count = 0;
  public JoystickRumble(Joystick j) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   // joystick = j;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  joystick.setRumble(RumbleType.kLeftRumble, 1);
  joystick.setRumble(RumbleType.kRightRumble, 1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    count++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(count > 20){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
