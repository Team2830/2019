/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveStraightForward extends Command {

  public DriveStraightForward() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  //  DriveTrain.talonRight.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
  //  DriveTrain.talonLeft.follow(DriveTrain.talonRight, FollowerType.AuxOutput1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.driveForward();
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
