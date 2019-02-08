/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * This is a subsystem that manipulates the hatch game piece.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  /*static Solenoid hatchSolenoid = new Solenoid(3);
  static Spark hatchIntake = new Spark(1);
  static DigitalInput forwardHatchLimitSwitch = new DigitalInput(0);
  static DigitalInput backwardsHatchLimitSwitch = new DigitalInput(1);
*/
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Acquires the object: sets the solenoid (pneumatics) to run and sets motor to run
   public void hatchAquire(){
    hatchSolenoid.set(true);
    hatchIntake.set(.5);
  }
 */
  
  /**
   * Scores the object: stops the solenoid (pneumatics)
  public void hatchScore(){
    hatchSolenoid.set(false);
  }
 */
  
  /**
   * Sets the motor to move the hatch down
  public void hatchDown(){
    hatchIntake.set(-.5);
  }
 */
  
  /**
   * Get status of the ForwardHatch limit switch
  public boolean isForwardHatchLimitSwitchHit(){
    return ! forwardHatchLimitSwitch.get();
  }
 */
  
  /**
   * Get status of the BackwardsHatch limit switch
  public boolean isBackwardsHatchLimitSwitchHit(){
    return ! backwardsHatchLimitSwitch.get();
  }
 */
  
  /**
   * Stops the hatch motor
  public void stopHatchIntake(){
    hatchIntake.stopMotor();
  }
  */
}
  
