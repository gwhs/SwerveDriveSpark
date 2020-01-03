/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.subsystems;

import org.usfirst.frc.team5507.robot.Robot;
import org.usfirst.frc.team5507.robot.commands.ShowLimelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final double REFRESH_RATE = .02;
  public static final int HATCH_CAM = 0;
  public static final int CARGO_CAM = 1;
  public static String cameraMode = "hatch";
  private static int camMode = 0;
  public static NetworkTable hatchTable = NetworkTableInstance.getDefault().getTable("limelight-hatch"); //11
  public static NetworkTable cargoTable = NetworkTableInstance.getDefault().getTable("limelight-cargo"); //12
  public static NetworkTable curNetworkTable = hatchTable;
  private static int currCam = 0;

  public static NetworkTableEntry tx = curNetworkTable.getEntry("tx");
  public static NetworkTableEntry ty = curNetworkTable.getEntry("ty");
  public static NetworkTableEntry ta = curNetworkTable.getEntry("ta");
  public static NetworkTableEntry tv = curNetworkTable.getEntry("tv");

  // read values periodically
  public static double limelightx;
  public static double limelighty;
  public static double limelightarea;
  public static double angleErr;
  public static boolean isView;
  private double kP = 0.05; // .06
  private double kI = 0.00138; 
  private double kD = 0.001;
  private double rkP = 0.0038;
  private double rkI = 0.006;
  private double rkD = 0.001;
  private double xErr = 0;
  private Double prevXErr;
  private double xIErr = 0;
  private double dRotation = 0;
  private double rIErr = 0;
  private boolean happy = false; // stop aligning x once happy is true
  private double xOffset = -.7;
  private double rotation = 0.0;
  private double strafe = 0;
  private double forward = 0.35;
  private Double currAngleErr; // starts as null
  private Double prevAngleErr; // starts as null

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ShowLimelight());
  }

  public void resetHappy()
  {
    happy = false;
    xIErr = 0;
    rIErr = 0;
    Robot.swerveDriveSubsystem.setSpeedMultiplier(1);
  }

  public static void setCamera(int camera) {
    if(camera == CARGO_CAM) {
      curNetworkTable = cargoTable;
      currCam = 1;
      cameraMode = "cargo";
    }
    else if (camera == HATCH_CAM) {
      curNetworkTable = hatchTable;
      currCam = 0;
      cameraMode = "hatch";
    }
    else {
      throw(new IllegalArgumentException("Camera out of range: " + camera));
    }
  }

  public int getCameraMode() {
    if(curNetworkTable == hatchTable) {
      return HATCH_CAM;
    }
    else {
      return CARGO_CAM;
    }
  }
  
  public  void switchModes() {
    if (camMode == 0) {
      curNetworkTable.getEntry("camMode").setNumber(1);
      camMode = 1;
    } 
    else {
      curNetworkTable.getEntry("camMode").setNumber(0);
      camMode = 0;
    }
  }

  public void toggleCamera(){
    if(currCam == 0) {
      curNetworkTable = cargoTable;
      currCam = 1;
      cameraMode = "cargo";
      tx = curNetworkTable.getEntry("tx");
      ty = curNetworkTable.getEntry("ty");
      ta = curNetworkTable.getEntry("ta");
      tv = curNetworkTable.getEntry("tv");
    }
    else{
      curNetworkTable = hatchTable;
      currCam = 0;
      cameraMode = "hatch";
      tx = curNetworkTable.getEntry("tx");
      ty = curNetworkTable.getEntry("ty");
      ta = curNetworkTable.getEntry("ta");
      tv = curNetworkTable.getEntry("tv");
    }
  }

  public void printInfo() {
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", limelightx);
    SmartDashboard.putNumber("LimelightY", limelighty);
    SmartDashboard.putNumber("LimelightArea", limelightarea);
    SmartDashboard.putBoolean("Is in view", isView);
    SmartDashboard.putNumber("Gyro Angle", (Robot.swerveDriveSubsystem.getGyroAngle()));
    SmartDashboard.putString("Camera Mode", cameraMode);
    SmartDashboard.putBoolean("Happy", happy);
  }

  public boolean getHappy()
  {
    return happy;
  }

  public double getStrafe() {
    xErr = 0 - limelightx;
    xIErr += (xErr)*REFRESH_RATE;
    double dXerr = 0;
    if(prevXErr != null)
    {
     dXerr = (xErr - prevXErr)/REFRESH_RATE;    
    }

    if (Math.abs(limelightx) > .5) {
      strafe = (kP*xErr) + (kI*xIErr) + (kD*dXerr);
    } else {
      strafe = 0;
    }
    prevXErr = xErr;
    if(getHappy()) {
      strafe = 0;
    }
    return strafe;
  }

  public double getRotation() {
    if(Math.abs(angleErr) < 4) {
      rIErr += angleErr * REFRESH_RATE;
    }
    else {
      if(angleErr < 0) {
        rIErr += -4*REFRESH_RATE;
      }
      else {
        rIErr += 4*REFRESH_RATE;
      }
    }
    if(Math.abs(angleErr) > 1.5 && Math.abs(limelightx) < 15) {
      rotation = (rkP * angleErr) + (dRotation * rkD) + rIErr*rkI;
    }
    
    return rotation;
  }

  public void setHappy() {
    // check if we are close enough and aligned to the target
    if(limelightarea > 15) {
      forward = 0;
      if(Math.abs(angleErr) < 5 && Math.abs(limelightx) < 2.0) {
        happy = true;
      }
    }
  }

  public double getAngleErr(double targetAngle) {
    double angleErr;
    double moddedGyro = (Robot.swerveDriveSubsystem.getGyroAngle() % 360);
    if(moddedGyro < 0) {
      if(Math.abs((targetAngle - moddedGyro - 360)) < Math.abs((targetAngle - moddedGyro))) {
        angleErr = (targetAngle - moddedGyro - 360);
      }
      else {
        angleErr = targetAngle - moddedGyro;
      }
    }
    else {
      if(Math.abs((targetAngle - moddedGyro + 360)) < Math.abs((targetAngle - moddedGyro))) {
        angleErr = (targetAngle - moddedGyro + 360);
      }
      else {
        angleErr = targetAngle - moddedGyro;
      }
    }
    return angleErr;
  }

  public double setdRotation(double targetAngle)
  {
    Double var = currAngleErr;
    currAngleErr = Robot.m_Limelight.getAngleErr(targetAngle);
    if(var != null)
    {
      prevAngleErr = var;
      return (currAngleErr - prevAngleErr) / REFRESH_RATE;
    }
    return 0;
  }


  public void align(double targetAngle, boolean cargo) //check the values on limelight
  {
    
    angleErr = this.getAngleErr(targetAngle);
    Robot.swerveDriveSubsystem.setIsAuto(true); 
    Robot.swerveDriveSubsystem.setSpeedMultiplier(.5);

    limelightx = tx.getDouble(0.3);
    limelightx += xOffset; // add target x offset for alignment errors

    limelighty = ty.getDouble(0.3);
    limelightarea = ta.getDouble(0.3);
    isView = tv.getBoolean(true);

    dRotation = Robot.m_Limelight.setdRotation(targetAngle);
    forward = 0.35;

    setHappy();
    strafe = -getStrafe();
    rotation = getRotation();
    if(getHappy())
    {
      forward = 0.35;
    }

    if(cargo) {
      strafe *= -1;
      forward *= -1;
    }
    // don't rotate if the target is not in view
    if(isView) {
      Robot.swerveDriveSubsystem.holonomicDrive(forward, strafe, rotation);
    }
    else {
      Robot.swerveDriveSubsystem.holonomicDrive(forward, -strafe, 0);
    } 
  }
}
