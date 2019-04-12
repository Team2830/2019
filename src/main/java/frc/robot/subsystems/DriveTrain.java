/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowTrajectory;

/**
 * TODO: Need Documentation Here
 */
public class DriveTrain extends Subsystem implements PIDOutput {

  public static WPI_TalonSRX talonLeft;
  public static WPI_TalonSRX talonRight;
  static WPI_VictorSPX victorLeft;
  static WPI_VictorSPX victorRight;
  static AHRS ahrs;
  public static DifferentialDrive m_drive;
  int collisionCount = 0;
  boolean collisionDetected = false;
  PIDController turnController;
  boolean startingVisionDrive = false;
  double visionYaw, visionX, visionY;
  double rotatePower = 0;
  double visionDriveX = 0;
  double visionDriveY = 0;

  public static int MotionMagicLoop = 0;
  public static int TimesInMotionMagic = 0;
  final static double kCollisionThreshold_Delta6 = 2f;

  public static final int LEFT_ENCODER = 0;
	public static final int RIGHT_ENCODER = 1;

  static int kPIDLoopIdx = 0;
  static int kSlotIdx = 0;
  static int kTimeoutMs = 30;
  static double kP = 0.2;
  static double kI = 0.0;
  static double kD = 0.0;
  static double peakOutput = 1.0;
  static int distance = 0;

  Vision vision;
  boolean m_LimelightHasValidTarget=  false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;

  /**
   * Setting up logging to the Shuffleboard
   */
 // private ShuffleboardTab shuffleboardTab;
  // private NetworkTableEntry leftEncoderEntry =
  // shuffleboardTab.add("Left Encoder", 0)
  //         .getEntry();
  NetworkTableEntry leftEncoderEntry, leftVelocityEntry, rightEncoderEntry, rightVelocityEntry;
  /**
   * TODO: Need Documentation Here (what happens in the constructor)
   */
  public DriveTrain() {
    talonLeft = new WPI_TalonSRX(15);
    talonRight = new WPI_TalonSRX(20);
    victorLeft = new WPI_VictorSPX(14);
    victorRight = new WPI_VictorSPX(21);
    ahrs = new AHRS(SPI.Port.kMXP);
  
    talonLeft.configFactoryDefault();
    talonRight.configFactoryDefault();
    talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    
    talonLeft.configContinuousCurrentLimit(20, 0);
    talonLeft.configPeakCurrentLimit(35, 0);
    talonLeft.configPeakCurrentDuration(100, 0);
    talonLeft.enableCurrentLimit(true);
    talonLeft.configOpenloopRamp(0.2, 0);

    talonRight.configContinuousCurrentLimit(20, 0);
    talonRight.configPeakCurrentLimit(35, 0);
    talonRight.configPeakCurrentDuration(100, 0);
    talonRight.enableCurrentLimit(true);
    talonRight.configOpenloopRamp(0.2, 0);

    talonRight.setInverted(true);
    talonLeft.setInverted(false);
    talonRight.setSensorPhase(true);
    talonLeft.setSensorPhase(true);
    victorLeft.follow(talonLeft);
    victorRight.follow(talonRight);
    victorLeft.setInverted(InvertType.FollowMaster);
    victorRight.setInverted(InvertType.FollowMaster);
    turnController = new PIDController(0.03, 0, 0, 0, ahrs, this);
    turnController.setInputRange (-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(2.0f);

    m_drive = new DifferentialDrive(talonLeft, talonRight);
    m_drive.setRightSideInverted(false);
    m_drive.setSafetyEnabled(false);
    vision = Robot.vision;
    resetCounters();
   // shuffleboardTab = Shuffleboard.getTab("Drivetrain");
   ShuffleboardLayout leftDriveTrainList = Shuffleboard.getTab("Drive Train")
    .getLayout("Left", BuiltInLayouts.kList)
    .withSize(10,3)
    .withPosition(0,0)
    .withProperties(Map.of("Label position", "LEFT"));
    ShuffleboardLayout rightDriveTrainList = Shuffleboard.getTab("Drive Train")
    .getLayout("Right", BuiltInLayouts.kList)
    .withSize(10,3)
    .withPosition(0,3)
    .withProperties(Map.of("Label position", "LEFT")); 
    ShuffleboardLayout driveMappingList = Shuffleboard.getTab("Drive Train")
    .getLayout("Mapping", BuiltInLayouts.kList)
    .withSize(2,3)
    .withPosition(10,3)
    .withProperties(Map.of("Label position", "LEFT"));
  // leftDriveTrainList.add("talonLeft", talonLeft).withWidget(BuiltInWidgets.kSpeedController);

    leftEncoderEntry = leftDriveTrainList.add("Position",0).withWidget(BuiltInWidgets.kGraph).withPosition(0,0).getEntry();
    leftVelocityEntry = leftDriveTrainList.add("Velocity",0).withWidget(BuiltInWidgets.kGraph).withPosition(0,1).getEntry();
    Shuffleboard.getTab("Drive Train").add("Gyro",ahrs).withWidget(BuiltInWidgets.kGyro);
    rightEncoderEntry = rightDriveTrainList.add("Position",0).withWidget(BuiltInWidgets.kGraph).withPosition(0,0).getEntry();
    rightVelocityEntry = rightDriveTrainList.add("Velocity",0).withWidget(BuiltInWidgets.kGraph).withPosition(0,1).getEntry();
   
    driveMappingList.add("Left Talon CAN",talonLeft.getDeviceID());
    driveMappingList.add("Right Talon CAN",talonRight.getDeviceID());
    driveMappingList.add("Left Victor CAN",victorLeft.getDeviceID());
    driveMappingList.add("Right Victor CAN",victorRight.getDeviceID());

    
    // Shuffleboard.getTab("Drive Train").add("Drive",m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public double joystickDeadband =0.04;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new DriveCommand());
  }

  /**
   * Has the robot drive in a certain style.
   * @param joystick
   */
  public void tankDrive(XboxController driverController){
    m_drive.tankDrive(-driverController.getY(GenericHID.Hand.kRight),-driverController.getY(GenericHID.Hand.kLeft));
    //TODO: remove debugging SmartDashboard calls
    // SmartDashboard.putNumber("left Tank",-joystick.getRawAxis(5));
    // SmartDashboard.putNumber("right Tank",-joystick.getRawAxis(1));
  }

  /**
   * Has the robot drive in another certain style.
   */
  public void arcadeDrive(XboxController driverController){
    double throttle = deadbanded((-1*driverController.getTriggerAxis(GenericHID.Hand.kLeft))+driverController.getTriggerAxis(GenericHID.Hand.kRight), joystickDeadband);
    double steering = deadbanded(driverController.getX(GenericHID.Hand.kLeft), joystickDeadband);
    //TODO: remove debugging SmartDashboard calls
    // SmartDashboard.putNumber("Throttle", throttle);
    // SmartDashboard.putNumber("Steering", steering);
    m_drive.arcadeDrive(throttle, steering);
  }

  /**
   * Has the robot drive in a different style.
   * 
    * @param driverController
   */
  public void curvatureDrive(XboxController driverController){
    double throttle = deadbanded((-1*driverController.getTriggerAxis(GenericHID.Hand.kLeft)+driverController.getTriggerAxis(GenericHID.Hand.kRight)), joystickDeadband);
    double steering = deadbanded(-driverController.getX(GenericHID.Hand.kLeft), joystickDeadband);
    m_drive.curvatureDrive(throttle, steering, driverController.getBButton());
  }

  /**
   * TODO: Need Documentation Here
   * 
   * @param joystick
   * @return double[] 3 values, the direction, the leftSpeed and the right speed
   */
  public double[] drive(XboxController driverController){
    turnController.disable();
    startingVisionDrive = true;
    double throttle = deadbanded((-1*driverController.getTriggerAxis(GenericHID.Hand.kLeft)+driverController.getTriggerAxis(GenericHID.Hand.kRight)), joystickDeadband);
    if (Math.abs(throttle) > 1){
      throttle = Math.copySign(1, throttle);
    }
    double steering = 0.6*deadbanded(-driverController.getX(GenericHID.Hand.kRight), joystickDeadband); 
    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(steering)), throttle);
    double speed, direction;
    if (throttle >= 0){
      if(steering >= 0){
        direction = 0;
        speed = throttle-steering;
      }else{
        direction = 1;
        speed = throttle+steering;
      }

    }else{
      if(steering >= 0){
        direction = 2;
        speed = throttle+steering;
      }else{
        direction = 3;
        speed = throttle-steering;
      }
    }
    
    double[] set = {direction,speed,maxInput};
    DetectCollision();
    if (collisionDetected == true){
      if (collisionCount < 5){
        speed = 0;
        maxInput = 0;
        collisionCount++;
      }
      else {
        collisionDetected = false;
        collisionCount = 0;
      }
    }
    return set;
  }

  /**
   *  Vision Drive used to turn based off vision target location
   * 
   * @param joystick
   */
  public void visionDrive(XboxController driverController){
    Update_Limelight_Tracking();
    double throttle = deadbanded((-1*driverController.getTriggerAxis(GenericHID.Hand.kLeft)+driverController.getTriggerAxis(GenericHID.Hand.kRight)), joystickDeadband);
    m_drive.arcadeDrive(throttle, m_LimelightSteerCommand);
  }

  public void Update_Limelight_Tracking(){
    final double STEER_K = 0.07;
    final double DRIVE_K = 0.26;
    final double DESIRED_TARGET_AREA = 13.0;
    final double MAX_DRIVE = 0.7;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (!Robot.vision.hasTargets()){
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }
    m_LimelightHasValidTarget = true;

    double steer_cmd = 0;
    if (Robot.vision.getDistance() >= 16){
      steer_cmd = 0.3;
      Math.copySign(steer_cmd, tx);  
    }

    // Start with proportional steering
    /*double steer_cmd = tx * STEER_K;
    if (Robot.vision.getDistance() < 16){
      steer_cmd = 0;
    }*/
    
    m_LimelightSteerCommand = steer_cmd;
   // SmartDashboard.putNumber("steerValue",m_LimelightSteerCommand);
    
    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into goal.
    if (drive_cmd > MAX_DRIVE){
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;
  }

  /**
   * Resets gyro and encoders everytime the robot is enabled,
   * Used for making accurate measurements.
   */
  public void resetCounters(){
    ahrs.zeroYaw();
    talonLeft.setSelectedSensorPosition(0, 0, 10);
    talonRight.setSelectedSensorPosition(0, 0, 10);
  }

  /**
   * TODO: Need Documentation Here
   * 
   * @param inches
   */
  public void driveForward(double inches){
    resetCounters();
    talonRight.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    talonLeft.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    talonLeft.config_kP(kSlotIdx, kP, kTimeoutMs);
    talonRight.config_kP(kSlotIdx, kP, kTimeoutMs);
    talonLeft.config_kI(kSlotIdx, kI, kTimeoutMs);
    talonRight.config_kI(kSlotIdx, kI, kTimeoutMs);
    talonLeft.config_kD(kSlotIdx, kD, kTimeoutMs);
    talonRight.config_kD(kSlotIdx, kD, kTimeoutMs);

    distance = getPulsesFromInches(inches);
    talonRight.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, ahrs.getYaw());
    talonLeft.follow(DriveTrain.talonRight, FollowerType.AuxOutput1);
  }

  public int getEncoderValue(int encoder){
		if (encoder == LEFT_ENCODER){
			return talonLeft.getSelectedSensorPosition(0);
		}
		else if(encoder == RIGHT_ENCODER){
			return talonRight.getSelectedSensorPosition(0);
		}
		return 0;
	}

  /**
   * TODO: Need Documentation Here
   * 
   * @return
   */
  public boolean isDriveForwardComplete(){
    if(distance >= getSensorAverage()){
      return true;
    }
    return false;
  }

  public void stopDriving(){
		talonLeft.set(0);
		talonRight.set(0);
  }
  
  public void setRight(double output){
		talonRight.set(output);
  }
  
	public void setLeft(double output){
		talonLeft.set(output);
  }
  


  /**
   * TODO: Need Documentation Here
   * 
   * @return
   */
  public static int getSensorAverage(){
    return (talonLeft.getSelectedSensorPosition()+talonRight.getSelectedSensorPosition())/2;
  }

  /**
   *  Function to set input based off deadband 
   * @param input
   * @param deadband
   * @return
   */
  public double deadbanded(double input, double deadband){
    if(Math.abs(input)>Math.abs(deadband)){
      return input;
    }else{
      return 0;
    }
  }
  
  /**
   * Makes sure that when the robot hits something that it won't keep going.
   */
  public void DetectCollision(){
    double lastWorldLinearAccelX = 0;
    double lastWorldLinearAccelY = 0;

    double currWorldLinearAccelX = ahrs.getWorldLinearAccelX();
    double currentJerkX = currWorldLinearAccelX - lastWorldLinearAccelX;
    lastWorldLinearAccelX = currWorldLinearAccelX;
    double currWorldLinearAccelY = ahrs.getWorldLinearAccelY();
    double currentJerkY = currWorldLinearAccelY - lastWorldLinearAccelY;
    lastWorldLinearAccelY = currWorldLinearAccelY;

    if ((Math.abs(currentJerkX) > kCollisionThreshold_Delta6) ||
      (Math.abs(currentJerkY) > kCollisionThreshold_Delta6)) {
        collisionDetected = true;
    }
  }

  /**
	 * Checks the gyro against setAngle.
	 * 
	 * If the difference (gyro angle - set angle) is greater than 1, 
	 * turn the robot slightly to the left.
	 * 
	 * If the difference (gyro angle - set angle) is less than -1,
	 * turn the robot slightly to the right.
	 * 
	 * @param presetAngle The preset angle
	 * @param throttle The default driving speed
	 */
	public void driveStraight(double throttle){
		/**TODO remove the check for turncontroller and just force the disablePID. **/
		if(turnController.isEnabled() && this.onTarget()){
			this.disablePID();
		}
		if(!turnController.isEnabled()){ /**TODO remove this loop. just run it...*/
			if(throttle > 0){
				if (ahrs.getAngle() > 1){ 
					talonLeft.set(throttle*.60);
					talonRight.set(throttle);
				}else if(ahrs.getAngle()< -1){
					talonLeft.set(throttle);
					talonRight.set(throttle*.60);
				}else{
					talonLeft.set(throttle);
					talonRight.set(throttle);
				}
			}
			else{
				if (ahrs.getAngle() > 1){
					talonLeft.set(throttle);
					talonRight.set(throttle*.60);
				}else if(ahrs.getAngle()< -1){
					talonLeft.set(throttle*.60);
					talonRight.set(throttle);
				}else{
					talonLeft.set(throttle);
					talonRight.set(throttle);
				}
			}
		}
	}
    
  public int getPulsesFromInches(double inches){
		return (int)(240/Math.PI*inches);
	}

	public double getInchesFromPulses(double d){
		return d*Math.PI/240;
  }
  
  public double getAngle(){
    return ahrs.getAngle();
  }

  public double getDistance(){
		return(talonLeft.getSelectedSensorPosition(0) + talonRight.getSelectedSensorPosition(0))/2;
  }
  
  public double getAccelerationX(){
		return ahrs.getWorldLinearAccelX();
  }
  
	public double getAccelerationY(){
		return ahrs.getWorldLinearAccelY();
  }

  public void setOpenloopRamp(double rampTime){
		talonLeft.configOpenloopRamp(rampTime, 10);
		talonRight.configOpenloopRamp(rampTime, 10);
  }

  public void disablePID(){
		turnController.disable();
  }
  
	public boolean onTarget(){
		return turnController.onTarget();
	}

  /**
   *  Writes to the dashboard values (can be used for debugging)
   */
  public void writeToSmartDashboard(){
    leftEncoderEntry.setDouble(talonLeft.getSelectedSensorPosition());
    rightEncoderEntry.setDouble(talonRight.getSelectedSensorPosition());
    // rightEncoderEntry.setDouble(talonRight.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Right Encoder Velocity", talonRight.getSelectedSensorVelocity());
    leftVelocityEntry.setDouble(talonLeft.getSelectedSensorVelocity());
    rightVelocityEntry.setDouble(talonRight.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Left Talon Current", talonLeft.getOutputCurrent());
    // SmartDashboard.putNumber("Right Talon Current", talonRight.getOutputCurrent());
    // SmartDashboard.putNumber("LeftTalon Current", Robot.pdp.getCurrent(15));
    // SmartDashboard.putNumber("LeftVictor Current", Robot.pdp.getCurrent(14));
    // SmartDashboard.putNumber("RightTalon Current", Robot.pdp.getCurrent(1));
    // SmartDashboard.putNumber("RigtVictor Current", Robot.pdp.getCurrent(0));
    // SmartDashboard.putData(ahrs);
    // SmartDashboard.putNumber("Gyro", ahrs.getAngle());
    // SmartDashboard.putData(talonLeft);
    // SmartDashboard.putBoolean("Collision Detected", collisionDetected);
    // SmartDashboard.putNumber("Motor Output Precent", talonRight.getMotorOutputPercent());
    // SmartDashboard.putNumber("Closed Loop Error", talonRight.getClosedLoopError(kPIDLoopIdx));
    // SmartDashboard.putNumber("Rotate Power", rotatePower);
    // SmartDashboard.putData("turn controller",turnController);
  }
  
  public void pidWrite(double output){
    rotatePower = output;
  }

  public void configArcadeDrive() {
    /*
    talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talonRight.setInverted(false);
    victorRight.setInverted(false);
    talonLeft.setInverted(true);
    victorLeft.setInverted(true);
    talonRight.setSensorPhase(true);
    talonLeft.setSensorPhase(true);
    victorLeft.follow(talonLeft);
    victorRight.follow(talonRight);
    */
  }
}
