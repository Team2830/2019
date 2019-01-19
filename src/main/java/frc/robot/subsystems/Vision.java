/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
boolean hasTargets = false;
double horizontalOffset = 0;
double verticalOffset = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public double getVerticalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  public double getHorizontalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }
  public double hasTargets(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }
  public void  writeToSmartDashboard(){
    SmartDashboard.putNumber("LimelightHasTargets", hasTargets());
    SmartDashboard.putNumber("LimelightVerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("LimelightHorizontalOffset", getHorizontalOffset());
    }
  }
