/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.commands.CargoIntake;


/**
 * This is a subsystem that manipulates the cargo game piece.
 */
public class Cargo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static Solenoid solenoid = new Solenoid(1);
  static Spark cargoIntakeFront = new Spark(0);
  static Spark cargoIntakeBack = new Spark(1);
  
 
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //public void setDefaultCommand(new CargoIntake());
    setDefaultCommand(new CargoIntake());
    
  }

  /**
   * Sets the motors to intake the object into cargo
   */
  public void cargoIntakeIn(){
    cargoIntakeFront.set(.5);
    cargoIntakeBack.set(.5);
  }
  
  /**
   * Sets the motors to outtake the object out of cargo
   */
  public void cargoIntakeOut(){
    cargoIntakeFront.set(-.5);
    cargoIntakeBack.set(-.5);
  }

  /**
   * Stops the intake motors
   */
  public void stopCargoIntake(){
    cargoIntakeFront.stopMotor();
    cargoIntakeBack.stopMotor();
  }

  /**
   * Sets the solenoid (pneumatics) to run, making the cargo go up
   */
  public void cargoUp(){
    solenoid.set(true);
  }

  /**
   * Sets the solenoid (pneumatics) to stop, making the cargo go down
   */
  public void cargoDown(){
    solenoid.set(false);
  }
}
