// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Suprise extends SequentialCommandGroup {
  /** Creates a new Suprise. */
  public Suprise(Shooter m_shooter, Intake m_intake, AutoShoot m_autoshoot, CommandSwerveDrivetrain m_drivetrain, Command path, Command path2,
            Command path3) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.runOnce(() -> m_drivetrain.seedFieldRelative(TeamSelector.getTeamColor()?m_drivetrain.getStartingPath("Suprise-1 Red"):m_drivetrain.getStartingPath("Suprise-1"))),
                Commands.parallel(
                        Commands.run(() -> m_shooter.setPivotAngle()),
                        Commands.startEnd(() -> m_shooter.setShooterVelocity(), () -> m_shooter.stopShooter()),
                        Commands.sequence(
                                Commands.deadline(
                                        path,
                                        new HandleAutonShoot(m_intake, m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot, m_drivetrain),
                                Commands.deadline(
                                        path2,
                                        Commands.sequence(new HandleAutonShoot(m_intake, m_shooter), new HandleAutonShoot(m_intake, m_shooter)),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot, m_drivetrain),
                                Commands.deadline(
                                        path3,
                                        new HandleAutonShoot(m_intake, m_shooter),
                                        Commands.run(() -> m_shooter.targetAngle = 187)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot, m_drivetrain)
                                )));
    }
}
