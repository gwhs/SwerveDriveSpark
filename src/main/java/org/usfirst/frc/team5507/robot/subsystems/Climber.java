/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team5507.robot.Robot;
import org.usfirst.frc.team5507.robot.commands.ClimberMoveArms;
import org.usfirst.frc.team5507.robot.commands.ClimberStop;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {
  private static CANSparkMax arm1 = new CANSparkMax(17, MotorType.kBrushless);
  private static CANSparkMax arm2 = new CANSparkMax(19, MotorType.kBrushless);
  private static CANSparkMax hand = new CANSparkMax(15, MotorType.kBrushless);
  private static CANEncoder NEncoder1 = new CANEncoder(arm1);
  private static CANEncoder NEncoder2 = new CANEncoder(arm2);
  public static CANEncoder NEncoder3 = new CANEncoder(hand);
  private static CANPIDController NPidController1 = new CANPIDController(arm1);
  private static CANPIDController NPidController2 = new CANPIDController(arm2);
  public static CANPIDController NPidController3 = new CANPIDController(hand);
  private static CANDigitalInput arm2F = new CANDigitalInput(arm2, CANDigitalInput.LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
  private static CANDigitalInput arm2R = new CANDigitalInput(arm2, CANDigitalInput.LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
  private static CANDigitalInput arm1R = new CANDigitalInput(arm1, CANDigitalInput.LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
  private static CANDigitalInput arm1F = new CANDigitalInput(arm1, CANDigitalInput.LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
  private final double GEARBOX_RATIO = 400;
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClimberMoveArms());
  }

  public CANSparkMax getHand() {
    return hand;
  }
  
  public CANPIDController getPIDControllerArm1()
  {
    return NPidController1;
  }

  public CANPIDController getPIDControllerArm2()
  {
    return NPidController2;
  }

  public boolean getForwardLimit() // bottom
  {
    return arm2F.get();
  }

  public boolean getReverseLimit() // top
  {
    return arm2R.get();
  }
  public boolean getArm1ForwardLimit() //bottom
  {
    return arm1F.get();
  }

  public void driveBackArmDown() // Need To Test
  {
    if(getForwardLimit() == false)
    {
      arm2.set(0.2);
    }
    else
    {
      arm2.set(0);
    } 
  }

  public void printLimit()
  {
    SmartDashboard.putBoolean("Bottom Limit", getForwardLimit());
    SmartDashboard.putBoolean("Top Limit", getReverseLimit());
    SmartDashboard.putBoolean("Front Arm ", getArm1ForwardLimit());
  }

  public void printGyro()
  { //wrong axis. XY angle not Z angle
    SmartDashboard.putNumber("Get Angle", (double)Robot.swerveDriveSubsystem.mNavX.getAngle());
    SmartDashboard.putNumber("Pitch Angle", (double)Robot.swerveDriveSubsystem.mNavX.getPitch());
    SmartDashboard.putNumber("Yaw Angle", (double)Robot.swerveDriveSubsystem.mNavX.getYaw());
    SmartDashboard.putNumber("Roll Angle", (double)Robot.swerveDriveSubsystem.mNavX.getRoll());
    SmartDashboard.putNumber("Hand Position", NEncoder3.getPosition());

  }

  public void stop() { //when pressed
    arm1.set(0);
    arm2.set(0);
  }
  public void moveArms(double speed1, double speed2) {
    arm1.set(-speed1);
    arm2.set(speed2);
  }
    // kP = 0.2; 
    // kI = 1e-4;
    // kD = 1; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;

  public static void setPID(CANPIDController Controller,double p, double i, double d) //used in RobotInit
  {
    Controller.setP(p);
    Controller.setI(i);
    Controller.setD(d);
  }

  public void armOneZero()
  {
    NPidController1.setReference(0, ControlType.kPosition);
  }

  public void stopArm1() {
    arm1.stopMotor();
  }

  public void stopArm2() {
    arm2.stopMotor();
  }

  public void moveArm1(double speed) {
    arm1.set(speed);
  }

  public void moveArm2(double speed) {
    arm2.set(speed);
  }

  public void moveHand1(double speed)
  {
    hand.set(speed);
  }

  public void stopHand() {
    hand.set(0);
  }

   public void moveHandsPosition(double Position) {
     NPidController3.setP(.1);
     NPidController3.setI(.000001);
     NPidController3.setFF(0);
     NPidController3.setReference(Position, ControlType.kPosition);
   }
}

