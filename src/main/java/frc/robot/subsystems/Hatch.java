/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.operateHatch;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * This is a subsystem that manipulates the hatch game piece.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static DoubleSolenoid hatchSolenoid = new DoubleSolenoid(2, 6);
  static Spark hatchIntake = new Spark(2);
  boolean hatchIn = true;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new operateHatch());
  }

  /**
   * Acquires the object: sets the solenoid (pneumatics) to run and sets motor to run
   * */
   public void hatchAcquire(){
     //hatchDown();
     hatchOut();
     hatchUp();
     hatchIn();
  }
 
  
  /**
   * Scores the object: stops the solenoid (pneumatics)
   */
   public void hatchScore(){
     hatchOut();
     hatchDown();
     hatchIn();
     //hatchUp();
  }
  
  /**
   * Sets the motor to move the hatch down
   */
  public void hatchDown(){
    hatchIntake.set(-.5);
  }

  /**
   * Sets the motor to move the hatch up
   */
  public void hatchUp(){
    hatchIntake.set(.5);
  }

  /**
   * Run hatch intake motor to set speed
   */
  public void driveHatch(double speed){
    hatchIntake.set(speed);
  }
  
  /**
   * Brings hatch out (solenoid runs forward)
   */
  public void hatchOut(){
    hatchSolenoid.set(DoubleSolenoid.Value.kForward);
    hatchIn = false;
  }
  
  /**
   * Brings hatch in (solenoid runs reverse)
   */
  public void hatchIn(){
    hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    hatchIn = true;
  }
 
  
  /**
   * Stops the hatch motor
   **/
  public void stopHatchIntake(){
    hatchIntake.stopMotor();
  }  
}
  
