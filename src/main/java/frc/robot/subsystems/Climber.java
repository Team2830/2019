/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.operateClimber;

/**
 * This is a subsystem that manipulates the climber game piece.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static DoubleSolenoid frontSolenoid = new DoubleSolenoid(3, 4);
  static DoubleSolenoid backSolenoid = new DoubleSolenoid(5, 7);
  
  public Climber(){
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new operateClimber());
  }

  /**
   * Front up (front solenoid run forward)
   */
  public void frontUp(){
    frontSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  /**
   * Front down (front solenoid run reverse)
   */
  public void frontDown(){
    frontSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Back up (back solenoid run forward)
   */
  public void backUp(){
    backSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  /**
   * Back down (back solenoid run reverse)
   */
  public void backDown(){
    backSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}