package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import harkerrobolib.util.MathUtil;

public class EEManual extends Command {

    public EEManual () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        runTusk();
        runMain();
    }

    private void runTusk() {
        // EndEffector.getInstance().moveToPosition(Constants.EndEffector.ALGAE_HOLD_POSITION);
    }

    private void runMain() {
        if (EndEffector.getInstance().isBackTriggered() && !EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
        }
        else if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.REVERSE_INTAKE_SPEED);
        }
        else if (EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed( 0);
        }
        else if (EndEffector.getInstance().algaeIn())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.ALGAE_HOLD_SPEED);
        }
        else
        {
            EndEffector.getInstance().setMainSpeed(
            EndEffector.getInstance().getPassive() ? 
            Constants.EndEffector.INTAKE_CORAL_SLOW_SPEED : 0);
        }
    }


    public boolean isFinished ()
    {
        return false;
    }
}
