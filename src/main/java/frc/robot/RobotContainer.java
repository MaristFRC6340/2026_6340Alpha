// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import javax.lang.model.util.ElementScanner14;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Driver Controller init
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Operator Controller init
  final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private Trigger driverA = driverXbox.a();
  private Trigger driverB = driverXbox.b();
  private Trigger driverX = driverXbox.x();
  private Trigger driverY = driverXbox.y();
  private Trigger driverL = driverXbox.leftBumper();
  private Trigger driverR = driverXbox.rightBumper();
  private Trigger driverStart = driverXbox.start();
  private Trigger driverBack = driverXbox.back();
  private Trigger driverLTrigger = driverXbox.leftTrigger(); // i believe the parameter changes axis value needed
  private Trigger driverRTrigger = driverXbox.rightTrigger();
  private Trigger driverLStick = driverXbox.leftStick();
  private Trigger driverRStick = driverXbox.rightStick();
  private Trigger driverDpadUp = driverXbox.povUp();
  private Trigger driverDpadRight = driverXbox.povRight();
  private Trigger driverDpadDown = driverXbox.povDown();
  private Trigger driverDpadLeft = driverXbox.povLeft();

  private Trigger operatorA = driverPS4.cross();
  private Trigger operatorB = driverPS4.circle();
  private Trigger operatorX = driverPS4.square();
  private Trigger operatorY = driverPS4.triangle();
  private Trigger operatorL = driverPS4.L1();
  private Trigger operatorR = driverPS4.R1();
  private Trigger operatorStart = driverPS4.options();
  private Trigger operatorLTrigger = driverPS4.axisGreaterThan(3,.05);
  private Trigger operatorRTrigger = driverPS4.axisGreaterThan(4,.05);
  private Trigger operatorLStick = new Trigger(() -> Math.abs(driverPS4.getLeftY()) > .05);
  private Trigger operatorRStick = new Trigger(() -> Math.abs(driverPS4.getRightY()) > .05);
  private Trigger operatorDpadUp = driverPS4.povUp();
  private Trigger operatorDpadRight = driverPS4.povRight();
  private Trigger operatorDpadDown = driverPS4.povDown();
  private Trigger operatorDpadLeft = driverPS4.povLeft();
  // private Trigger operatorBack = driverPS4.create();


  private double rotationSpeed = 0.8;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
                    .withControllerRotationAxis(driverXbox::getRightX)
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.8) // change later
                    .scaleRotation(0.8)
                    .allianceRelativeControl(true); // in Alpha_2025, they make this false & flipDirection based on alliance instead 

  SwerveInputStream driveAngularSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -.5, // Set these values for slower: 1/2 Speed
                    () -> driverXbox.getLeftX() * -.5) // Set these values for slower: 1/2 Speed
                    .withControllerRotationAxis(driverXbox::getRightX)
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.3)
                    .scaleRotation(0.25)
                    .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .scaleRotation(rotationSpeed)
                                                                    .allianceRelativeControl(true);
  
  SwerveInputStream driveAngularVelocityKeyboardSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.3)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                  .withControllerHeadingAxis(() ->
                                                  Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                  () ->
                                                  Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI *2))
                                                  .headingWhile(true)
                                                  .translationHeadingOffset(true)
                                                  .translationHeadingOffset(Rotation2d.fromDegrees(0));

  // use SmartDashboard for a list of auto options
  SendableChooser<Command> autoChooser;

  // test
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // add auto options to SmartDashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    // Slow Mode Fix Temporary - Right Trigger Makes the Robot Drive Slow - michaudc
    driverRTrigger.whileTrue(drivebase.driveFieldOriented(driveAngularSlow));

    // X: Bump align (Driver)
    // driverX.onTrue(new InstantCommand(() -> ));
    
    // debug
    // driverDpadUp.onTrue(new InstantCommand(() -> turretSubsystem.getSetPositionCommand(0.01)));
    // driverX.whileTrue(new InstantCommand(() -> turretSubsystem.getSetTransferSpeed(0.25)));
    driverX.whileTrue(new InstantCommand(() -> turretSubsystem.getSetFlywheelCommand(TurretConstants.flywheelSpeed)
    .withTimeout(1)
    .andThen(turretSubsystem.getSetTransferCommand(0.5))));
    // .andThen(turretSubsystem.getSetTransferCommand(0.5)));
    // driverX.whileTrue(Commands.waitUntil(() -> turretSubsystem.isFlywheelRunning()).withTimeout(0.5)
    // .andThen(turretSubsystem.getSetFlywheelCommand(TurretConstants.flywheelSpeed)));
    driverY.whileTrue(turretSubsystem.getSetTransferCommand(0.75)); // operator
    driverB.whileTrue(turretSubsystem.getSetFlywheelCommand(TurretConstants.flywheelSpeed));
    driverA.whileTrue(turretSubsystem.shootWhileHeld(TurretConstants.flywheelSpeed, TurretConstants.transferSpeed));

    driverRTrigger.whileTrue(turretSubsystem.getSetHoodAngleHigh());  
    driverLTrigger.whileTrue(turretSubsystem.getSetHoodAngleLow());  


    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("HubDepotTest");
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void onTeleopInit() {
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
  }
}
