// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.CenterAuton;
import frc.robot.Commands.ClimberCommand;
import frc.robot.Commands.FourPieceLeft;
import frc.robot.Commands.HandleAutonShoot;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed

  // Replace all instances of Nathan Speed with MaxSpeed for production code
  private double MaxAngularRate = 6.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController drivercontroller = new CommandXboxController(0); // drive controller
  private final CommandXboxController operatercontroller = new CommandXboxController(1); // operator controller
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Camera m_camera;
  private final TeamSelector m_teamSelector;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final AutoShoot m_autoshoot;
  private final Climber m_climber;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  //private final VisionPose m_visionpose;

  // Field centric drive
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 2% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // This is the auto that we run open the path planner app for mor details
  private Command runAuto = drivetrain.getAutoPath("4 Piece");

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive
        .withVelocityX(-drivercontroller.getLeftY() * Constants.MAX_SPEED) // Drive forward with negative Y (forward)
        .withVelocityY(-drivercontroller.getLeftX() * Constants.MAX_SPEED) // Drive left with negative X (left)
        .withRotationalRate(-drivercontroller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
      );

      drivercontroller.rightBumper().whileFalse(Commands.run(() -> m_shooter.targetAngle = 187));
      drivercontroller.rightBumper().whileTrue(
        Commands.parallel(drivetrain.applyRequest(() -> 
        drive
          .withVelocityX(-drivercontroller.getLeftY() * Constants.MAX_SPEED * (TeamSelector.getTeamColor()?-1:1)) // Drive forward with negative Y (forward)
          .withVelocityY(-drivercontroller.getLeftX() * Constants.MAX_SPEED * (TeamSelector.getTeamColor()?-1:1)) // Drive left with negative X (left)
          .withRotationalRate(m_autoshoot.targetAll(TeamSelector.getTeamColor()?4:7, ()->operatercontroller.y().getAsBoolean(), () -> operatercontroller.rightBumper().getAsBoolean())) // Drive counterclockwise with negative X (left)
        ), Commands.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d()))
      ));
    
    
    m_climber.setDefaultCommand(new ClimberCommand(m_climber, () -> operatercontroller.getLeftY(), () -> operatercontroller.getRightY(), () -> drivercontroller.leftTrigger(0.5).getAsBoolean()));

    drivercontroller.a().whileTrue(
      drivetrain.applyRequest(() -> brake)
    );

    operatercontroller.a().onTrue(Commands.run(()->m_shooter.enableShooter = true));
    operatercontroller.x().whileTrue(Commands.startEnd(() -> m_intake.autoIntake(), () -> m_intake.stopIntakeAndFeed()));
    operatercontroller.rightBumper().whileTrue(Commands.startEnd(() -> m_intake.intakeMotorPower(-0.6), () -> m_intake.intakeMotorPower(0.0)));
    drivercontroller.b().whileTrue(Commands.startEnd(() -> m_intake.feedMotorPower(0.6), () -> m_intake.feedMotorPower(0.0)));
    operatercontroller.leftBumper().whileTrue(Commands.race(Commands.startEnd(()->m_shooter.velocityRPM = 1000, ()->m_shooter.velocityRPM = 3000), Commands.startEnd(()->m_shooter.targetAngle = 217, ()->m_shooter.targetAngle = 187)));
    // reset the field-centric heading on left bumper press
    drivercontroller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    m_shooter = new Shooter();
    m_climber = new Climber();
    m_intake = new Intake();
    m_camera = new Camera(drivetrain);
    m_teamSelector = new TeamSelector();
    m_autoshoot = new AutoShoot(drivetrain, m_shooter, m_camera, m_intake, drive);

    NamedCommands.registerCommand("FeedOn", Commands.race(new WaitCommand(0.1), Commands.startEnd(()-> m_intake.feedMotorPower(0.5), ()-> m_intake.feedMotorPower(0))));
    NamedCommands.registerCommand("Enable Intake", new HandleAutonShoot(m_intake, m_shooter));
    autoChooser.addOption("Center", new CenterAuton(m_shooter, m_intake, m_autoshoot, drivetrain, drivetrain.getPath("Center-1"), drivetrain.getPath("Center-2"), drivetrain.getPath("Center-3"), drivetrain.getPath("Center-4"), drivetrain.getPath("Center-5")));
    autoChooser.addOption("A Four Piece", new FourPieceLeft(m_shooter, m_intake, m_autoshoot, drivetrain, drivetrain.getPath("4-2a Piece"), drivetrain.getPath("4-3 Piece"), drivetrain.getPath("4-4 Piece")));
    // autoChooser.addOption("Suprise", new Suprise(m_shooter, m_intake, m_autoshoot, drivetrain, drivetrain.getPath("Suprise-1"), drivetrain.getPath("Suprise-2"), drivetrain.getPath("Suprise-3")));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    
    // return runAuto;
    return autoChooser.getSelected();
    // return new CenterAuton(m_shooter, m_intake, m_autoshoot, drivetrain, drivetrain.getPath("Center-1"), drivetrain.getPath("Center-2"), drivetrain.getPath("Center-3"), drivetrain.getPath("Center-4"), drivetrain.getPath("Center-5"));
  }
  
}
