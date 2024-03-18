// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.TeamSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceLeft extends SequentialCommandGroup {
    /** Creates a new SimpleTwoPieceAuton. */
    public FourPieceLeft(Shooter m_shooter, Intake m_intake, AutoShoot m_autoshoot,
            CommandSwerveDrivetrain m_drivetrain, Command path, Command path2,
            Command path3, Command path4) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                Commands.runOnce(
                        () -> m_drivetrain
                                .seedFieldRelative(TeamSelector
                                        .getTeamColor() ? m_drivetrain.getStartingPath("4-2a Red Piece")
                                                : m_drivetrain.getStartingPath("Amp Side-1"))),
                Commands.parallel(
                        Commands.run(() -> m_shooter.setPivotAngle()),
                        Commands.runEnd(() -> m_shooter.setShooterVelocity(),
                                () -> m_shooter.stopShooter()),
                        Commands.sequence(
                                new WaitCommand(0.1),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot,
                                        m_drivetrain),
                                Commands.deadline(
                                        Commands.race(Commands.sequence(path,
                                                new WaitCommand(1.25)),
                                                Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake,
                                                m_shooter)),

                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot,
                                        m_drivetrain),
                                Commands.deadline(
                                        Commands.race(path2,
                                                Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake, m_shooter)),
                                Commands.deadline(
                                        Commands.race(path3,
                                                Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake,
                                                m_shooter)),
                                new AlignAndShoot(m_shooter, m_intake, m_autoshoot, m_drivetrain),
                                Commands.deadline(
                                        Commands.race(path4,
                                                Commands.run(() -> m_shooter.targetAngle = 190)),
                                        new HandleAutonShoot(m_intake, m_shooter)))));
    }
}
