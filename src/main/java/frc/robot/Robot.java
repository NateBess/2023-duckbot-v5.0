// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// import edu.wpi.first.wpilibj.Compressor;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    // private XboxController xbController = new XboxController(0);
    private XboxController xbControllerTwo = new XboxController(1);

    private VictorSP armMotor = new VictorSP(0); // 0 is the RIO PWM port this is connected to


    // Claw open and close.
    private DoubleSolenoid doubleSolenoidOne = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    // Level 1 Solenoid
    private DoubleSolenoid doubleSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    // Level 2 Solenoid
    private DoubleSolenoid doubleSolenoidThree = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

    // Level 3 Solenoid
    //private DoubleSolenoid doubleSolenoidFour = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    // This is where you program buttons on the xbox controller...
    @Override
    public void teleopPeriodic() {

        // Claw open / close if statements.
        if (xbControllerTwo.getLeftBumperPressed()) {
            doubleSolenoidOne.set(kForward);
          }
        if (xbControllerTwo.getRightBumperPressed()) {
            doubleSolenoidOne.set(kReverse);
          }
        
        // Level 1 & 2 Buttons
        if (xbControllerTwo.getYButtonPressed()) {
            doubleSolenoidTwo.set(kForward);
            doubleSolenoidThree.set(kForward);
            
          }
        if (xbControllerTwo.getAButtonPressed()) {
            doubleSolenoidTwo.set(kReverse);
            doubleSolenoidThree.set(kReverse);
          }

        if (xbControllerTwo.getBButtonPressed()) {
            doubleSolenoidTwo.set(kReverse);
            doubleSolenoidThree.set(kForward);
        }

        if ((xbControllerTwo.getLeftTriggerAxis() > 50) && (xbControllerTwo.getRightStickButtonPressed())) {
            armMotor.set(-0.20);
        }
        if ((xbControllerTwo.getRightTriggerAxis() > 50) && (xbControllerTwo.getRightStickButtonPressed())) {
            armMotor.set(0.20);
        }
        if ((xbControllerTwo.getRightTriggerAxis() < 50) && (xbControllerTwo.getLeftTriggerAxis() < 50)) {
            armMotor.set(0.00);
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
