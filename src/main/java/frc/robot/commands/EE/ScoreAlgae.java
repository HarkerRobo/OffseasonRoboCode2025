package frc.robot.commands.EE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class ScoreAlgae extends Command
{
    Timer timer;
    public ScoreAlgae()
    {
        addRequirements(EndEffector.getInstance());
        timer = new Timer();
        timer.start();
    } 

    @Override
    public void initialize()
    {
        timer.reset();
        EndEffector.getInstance().setPassive(false);
        EndEffector.getInstance().setMainSpeed(0);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(1.0);
    }

    @Override
    public void end(boolean interrupted)
    {
        EndEffector.getInstance().setPassive(true);
    }
}
