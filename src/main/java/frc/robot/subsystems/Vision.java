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
 * TODO: Need Documentation Here
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  boolean hasTargets = false;
  double horizontalOffset = 0;
  double verticalOffset = 0;

  public Vision(){
   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * TODO: Need Documentation Here
   */
  public double getVerticalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  /**
   * TODO: Need Documentation Here
   * @return
   */
  public double getHorizontalOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  /**
   * TODO: Need Documentation Here
   */
  public boolean hasTargets(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1)
      return true;
    return false;
  }

  /**
   * TODO: Need Documentation Here
   */
  public double getDistance(double h2){
    double a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double a1= 5.6;
    double h1= 10.5;
   return (h2-h1)/(Math.tan((a1+a2)*Math.PI/180));
  }

  
  public void writeToSmartDashboard(){
    SmartDashboard.putBoolean("LimelightHasTargets", hasTargets());
    SmartDashboard.putNumber("LimelightVerticalOffset", getVerticalOffset());
    SmartDashboard.putNumber("LimelightHorizontalOffset", getHorizontalOffset());
    SmartDashboard.putNumber("LimelightGetDistance", getDistance(25.625));
  }
}