/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.operateClimber;

/**
 * This is a subsystem that manipulates the climber game piece.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static Spark rightFront = new Spark(6);
  static Spark leftFront = new Spark(4);
  static Spark rightBack = new Spark(3);
  static Spark leftBack = new Spark(5);
  static DoubleSolenoid solenoid = new DoubleSolenoid(4, 5);

  public Climber(){
    leftFront.setInverted(true);
    leftBack.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new operateClimber());
  }

  /**
   * Sets the motors to climb up
   */
   public void forwardClimber(){
    leftFront.set(.9);
    rightFront.set(.9);
    leftBack.set(.9);
    rightBack.set(.9);
  }
  
  /**
   * Sets the motors to climb down
   */
   public void backwardsClimber(){
    leftFront.set(-.9);
    rightFront.set(-.9);
    leftBack.set(-.9);
    rightBack.set(-.9);
  }

  /**
   * Drives climber up/down based off joystick by setting speeds of spark motors
   */
  public void driveClimber(Joystick joystick){
    double speed = joystick.getRawAxis(5);
    leftFront.set(speed);
    rightFront.set(speed);
    leftBack.set(speed);
    rightBack.set(speed);
  }
  
  /**
   * Stops motors
   */
  public void stopClimber(){
    leftFront.stopMotor();
    rightFront.stopMotor();
    leftBack.stopMotor();
    rightBack.stopMotor();
  }

  /**
   * Climber clamps (solenoid run forward)
   */
  public void clamp(){
    solenoid.set(DoubleSolenoid.Value.kForward);
  }
  
  /**
   * Climber unclamps (solenoid run reverse)
   */
  public void unClamp(){
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }
}