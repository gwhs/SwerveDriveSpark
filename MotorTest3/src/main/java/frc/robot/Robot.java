/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public CANSparkMax mDriveMotor1;
  public CANSparkMax mDriveMotor2;
  public CANEncoder mDriveEncoder1;
  public CANEncoder mDriveEncoder2;
  public CANPIDController mPIDController1;
  public CANPIDController mPIDController2;
  public XboxController XboxController1;
  public XboxController XboxController2;
  double pos1 = 0;
  double pos2 = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    mDriveMotor1 = new CANSparkMax(5, MotorType.kBrushless);
    mDriveMotor2 = new CANSparkMax(15, MotorType.kBrushless);
    mDriveEncoder1 = new CANEncoder(mDriveMotor1);
    mDriveEncoder2 = new CANEncoder(mDriveMotor2);
    mPIDController1 = new CANPIDController(mDriveMotor1);
    mPIDController2 = new CANPIDController(mDriveMotor2);
    XboxController1 = new XboxController(0);
    XboxController2 = new XboxController(1);
    mPIDController1.setP(.1);
    mPIDController2.setP(.1);
    mPIDController1.setD(1);
    mPIDController2.setD(1);
    mPIDController1.setI(.0000001);
    mPIDController2.setI(.0000001);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Motor 1 Position", mDriveEncoder1.getPosition());
    SmartDashboard.putNumber("Motor 2 Position", mDriveEncoder2.getPosition());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    pos1 = mDriveEncoder1.getPosition();
    pos2 = mDriveEncoder2.getPosition();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    if(XboxController1.getXButtonPressed())
    {
      mPIDController1.setReference(pos1+50, ControlType.kPosition);
      mPIDController2.setReference(pos2+50, ControlType.kPosition);
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (XboxController1.getXButton()) {
      mPIDController1.setReference(50, ControlType.kPosition);
      mPIDController2.setReference(50, ControlType.kPosition);
    }
     else {

      if (Math.abs(XboxController1.getRawAxis(0)) > 0.1) {
        mDriveMotor1.set(XboxController1.getRawAxis(0));
      } 
      else {
        mDriveMotor1.set(0);
      }
      if (Math.abs(XboxController1.getRawAxis(5)) > 0.1) {
        mDriveMotor2.set(XboxController1.getRawAxis(5));
      } 
      else {
        mDriveMotor2.set(0);
      }

    }

   
  }// if we're pressing the button set reference top 50

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
