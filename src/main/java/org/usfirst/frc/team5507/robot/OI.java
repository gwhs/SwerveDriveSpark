package org.usfirst.frc.team5507.robot;

import org.usfirst.frc.team5507.robot.commands.*;
import org.usfirst.frc.team5507.robot.input.DPadButton;
import org.usfirst.frc.team5507.robot.input.IGamepad;
import org.usfirst.frc.team5507.robot.input.XboxGamepad;
import org.usfirst.frc.team5507.robot.input.DPadButton.Direction;
import org.usfirst.frc.team5507.robot.subsystems.HolonomicDrivetrain;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private IGamepad mController = new XboxGamepad(0);
	private IGamepad climbController = new XboxGamepad(1);

	private Robot mRobot;

	public OI(Robot robot) {
		mRobot = robot;
	}

	public void registerControls() { 
		//main controls
		mController.getAButton().whileHeld(new CargoOut());
		mController.getAButton().whenReleased(new CargoIn());

		mController.getBButton().whileHeld(new HatchOut());
		mController.getBButton().whenReleased(new HatchIn());

		mController.getXButton().whileHeld(new AutoAlign(mRobot.getDrivetrain()));
		mController.getXButton().whenReleased(new ResetHappy());
		
		mController.getYButton().whenPressed(new ZeroNavX());


		mController.getRightBumperButton().whenPressed(new SwitchLedModes());
		mController.getLeftBumperButton().whenPressed(new ToggleCamera());

		mController.getStartButton().whenPressed(new AdjustSpeedMultiplier(.3));
		mController.getBackButton().whenPressed(new AdjustSpeedMultiplier(.825));

		// mController.getStartButton().whenPressed(new SetFieldOriented(mRobot.getDrivetrain(),true));
		// mController.getBackButton().whenPressed(new SetFieldOriented(mRobot.getDrivetrain(), false));

		
		mController.getDPadButton(Direction.RIGHT).whenPressed(new AdjustFieldOrientedAngleCommand(mRobot.getDrivetrain(), true));
		mController.getDPadButton(Direction.LEFT).whenPressed(new AdjustFieldOrientedAngleCommand(mRobot.getDrivetrain(), false));

		//climber controls
		
		//climbController.getRightBumperButton().whenPressed(new MoveHandsDistance(122.2));

		climbController.getRightBumperButton().whenReleased(new ClimberMoveHand(.7));
		climbController.getRightBumperButton().whenReleased(new ClimberMoveHand(0));

		climbController.getYButton().whenPressed(new HookForward());
		climbController.getBButton().whenPressed(new HookBack());
		
		climbController.getAButton().whileHeld(new PivotDown());
		climbController.getAButton().whenReleased(new PivotUp());

		//climbController.getXButton().whenPressed(new FrontArmMatch());
		//climbController.getXButton().whenPressed(new AutoClimb());
		climbController.getXButton().whenPressed(new WheelsDriveForwardFieldTest());
		//climbController.getXButton().whenPressed(new MoveHandsDistance(122.2));
		//climbController.getXButton().whenPressed(new FrontArmToLimit());
	}
	public IGamepad getController() {
		return mController;
	}

	public IGamepad getClimberController() {
		return climbController;
	}
}
