/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;

public class operateClimber extends CommandGroup {
  public operateClimber() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  public void execute() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

   /** addSequential(new ClimberFrontUp());
    addSequential(new DriveDistance(11.625));
    addSequential(new ClimberBackUp());
    addSequential(new ClimberFrontDown());
    addSequential(new DriveDistance(5.125));
    addSequential(new ClimberBackDown());
    addSequential(new DriveDistance(10));*/
    Joystick driverStick = Robot.oi.getDriverJoystick();

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
