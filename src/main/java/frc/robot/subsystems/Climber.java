/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is a subsystem that manipulates the climber game piece.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static Solenoid solenoid = new Solenoid(4);
  static Spark frontRightClimber = new Spark(2);
  static Spark frontLeftClimber = new Spark(3);
  static Spark backRightClimber = new Spark(4);
  static Spark backLeftClimber = new Spark(5);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Sets the motors to climb up
   */
  public void forwardClimber(){
    frontLeftClimber.set(.9);
    frontRightClimber.set(.9);
    backLeftClimber.set(.9);
    backRightClimber.set(.9);
  }

  /**
   * Sets the motors to climb up
   */
  public void backwardsClimber(){
    frontLeftClimber.set(-.5);
    frontRightClimber.set(-.5);
    backLeftClimber.set(-.5);
    backRightClimber.set(-.5);
  }

  /**
   * Stops motors
   */
  public void stopClimber(){
    frontLeftClimber.stopMotor();
    frontRightClimber.stopMotor();
    backLeftClimber.stopMotor();
    backRightClimber.stopMotor();
  }
}
