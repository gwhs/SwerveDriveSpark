
package org.usfirst.frc.team5507.robot;

import org.usfirst.frc.team5507.robot.commands.AutoAlign;
import org.usfirst.frc.team5507.robot.commands.AutoGetOffHab1;
import org.usfirst.frc.team5507.robot.commands.AutoLv1Team2LeftHatch;
import org.usfirst.frc.team5507.robot.commands.AutoLv1Team2RightHatch;
import org.usfirst.frc.team5507.robot.subsystems.Cargo;
import org.usfirst.frc.team5507.robot.subsystems.Climber;
import org.usfirst.frc.team5507.robot.subsystems.HatchDelivery;
import org.usfirst.frc.team5507.robot.subsystems.Hook;
import org.usfirst.frc.team5507.robot.subsystems.Limelight;
import org.usfirst.frc.team5507.robot.subsystems.Pivot;
import org.usfirst.frc.team5507.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	public static final boolean DEBUG = true;
	
	
	public static SwerveDriveSubsystem swerveDriveSubsystem;
	private Timer timer; 
	public static Climber m_climber;
	public static HatchDelivery m_HatchDelivery;
	public static Limelight m_Limelight;
	public static Cargo m_cargo;
	public static Hook m_hook;
	public static Pivot m_pivot;
	public Compressor compressor;
	public static Command m_autoCommand;
	SendableChooser<Integer> m_autoChooser = new SendableChooser<>();
	SendableChooser<Integer> m_alignChooser = new SendableChooser<Integer>();
	public static AutoAlign m_align;
	private static OI mOI;
	public static double targetPos;
	public static boolean autoMode;
	
	public static OI getOI() {
		return mOI;
	}

	//Auto system
	private static final int AUTO_OFF_1 = 0;
	private static final int AUTO_OFF_2 = 1;
	private static final int AUTO_MIDDLE_LEFT_HATCH = 1;
	private static final int AUTO_MIDDLE_RIGHT_HATCH = 2;

	//Angle Chooser System
	private static final int FRONT_CARGO = 0;
	private static final int LEFT_CARGO = 90;
	private static final int LEFT_FRONT_ROCKET = -29; 
	private static final int LEFT_CENTER_ROCKET = -91; //Maybe
	private static final int LEFT_BACK_ROCKET = -151;
	private static final int RIGHT_CARGO = -90;
	private static final int RIGHT_FRONT_ROCKET = 29;
	private static final int RIGHT_CENTER_ROCKET = 91; // Maybe
	private static final int RIGHT_BACK_ROCKET = 151;
	private static final int LOADING_STATION = 180;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		compressor = new Compressor();
		swerveDriveSubsystem = new SwerveDriveSubsystem();
		m_Limelight = new Limelight();
		m_climber = new Climber();
		m_HatchDelivery = new HatchDelivery();
		m_cargo = new Cargo();
		m_hook = new Hook();
		m_pivot = new Pivot();
		timer = new Timer();	
		mOI = new OI(this);
		mOI.registerControls();
		swerveDriveSubsystem.zeroGyro();
		autoMode = false;

		compressor.setClosedLoopControl(true);

		m_alignChooser.setDefaultOption("Front Cargo Bay", FRONT_CARGO);
		m_alignChooser.addOption("Loading Station", LOADING_STATION);
		m_alignChooser.addOption("Left Cargo Bay", LEFT_CARGO );
		m_alignChooser.addOption("Right Cargo Bay", RIGHT_CARGO);
		m_alignChooser.addOption("Left Front Rocket", LEFT_FRONT_ROCKET);
		m_alignChooser.addOption("Left Center Rocket", LEFT_CENTER_ROCKET);
		m_alignChooser.addOption("Left Back Rocket", LEFT_BACK_ROCKET);
		m_alignChooser.addOption("Right Front Rocket", RIGHT_FRONT_ROCKET);
		m_alignChooser.addOption("Right Center Rocket", RIGHT_CENTER_ROCKET);
		m_alignChooser.addOption("Right Back Rocket", RIGHT_BACK_ROCKET);
		


		m_autoChooser.setDefaultOption("Get off hab zone", AUTO_OFF_1);
		m_autoChooser.addOption("Middle Left Hatch", AUTO_MIDDLE_LEFT_HATCH);
		m_autoChooser.addOption("Middle Right Hatch", AUTO_MIDDLE_RIGHT_HATCH);

		SmartDashboard.putData("Align Chooser", m_alignChooser);
		SmartDashboard.putData("Auto Chooser", m_autoChooser);

		targetPos = FRONT_CARGO;
							
		
	}

	@Override
	public void robotPeriodic() {
		m_climber.printLimit(); // prints limit switch values to SmartDashboard.
		SmartDashboard.putNumber("Adjusted Drivetrain Angle", swerveDriveSubsystem.getGyroAngle());
		SmartDashboard.putNumber("Raw Drivetrain Angle", swerveDriveSubsystem.getRawGyroAngle());
		SmartDashboard.putNumber("Drivetrain Rate", swerveDriveSubsystem.getGyroRate());
		SmartDashboard.putNumber("Gyro Update Rate", swerveDriveSubsystem.getNavX().getActualUpdateRate());
		for (int i = 0; i < 4; i++) {
			SmartDashboard.putNumber("Drive Current Draw " + i, swerveDriveSubsystem.getSwerveModule(i).getDriveMotor().getOutputCurrent());
			SmartDashboard.putNumber("Angle Current Draw " + i, swerveDriveSubsystem.getSwerveModule(i).getAngleMotor().getOutputCurrent());
		}
		targetPos = FRONT_CARGO;
		switch(m_alignChooser.getSelected())
		{
			case FRONT_CARGO:
				targetPos =  FRONT_CARGO;
				break;
			case LEFT_CARGO: 
				targetPos = LEFT_CARGO;
				break;
			case LEFT_FRONT_ROCKET: 
				targetPos = LEFT_FRONT_ROCKET;
				break;
			case LEFT_CENTER_ROCKET: 
				targetPos = LEFT_CENTER_ROCKET+1;
				break;		
			case LEFT_BACK_ROCKET: 
				targetPos = LEFT_BACK_ROCKET;
				break;	
			case RIGHT_CARGO: 
				targetPos = RIGHT_CARGO;
				break;	
			case RIGHT_FRONT_ROCKET: 
				targetPos = RIGHT_FRONT_ROCKET;
				break;
			case RIGHT_CENTER_ROCKET:
				targetPos = RIGHT_CENTER_ROCKET-1;
				break;	
			case RIGHT_BACK_ROCKET:
				targetPos = RIGHT_BACK_ROCKET;
				break;
			case LOADING_STATION: 
				targetPos = LOADING_STATION;
				break;

		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		for (int i = 0; i < 4; i++) {
			swerveDriveSubsystem.getSwerveModule(i).robotDisabledInit();
		}
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {

		swerveDriveSubsystem.zeroGyro();

		switch(m_autoChooser.getSelected())
		{
			case AUTO_OFF_1:
				m_autoCommand = new AutoGetOffHab1();
				break;
			case AUTO_MIDDLE_LEFT_HATCH:
				m_autoCommand = new AutoLv1Team2LeftHatch();
				break;
			case AUTO_MIDDLE_RIGHT_HATCH:
				m_autoCommand =  new AutoLv1Team2RightHatch();	
				break;
		}

		if (m_autoCommand != null) {
			//m_autoCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();		
	}

	@Override
	public void teleopInit() {
		swerveDriveSubsystem.setFieldOriented(true);
		

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		swerveDriveSubsystem.holonomicDrive(.3,0,0);
	}

	public SwerveDriveSubsystem getDrivetrain() {
		return swerveDriveSubsystem;
	}
}
