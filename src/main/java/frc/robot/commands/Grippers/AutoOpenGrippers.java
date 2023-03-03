package frc.robot.commands.Grippers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

    public class AutoOpenGrippers extends CommandBase {
    private Timer OpenGripTimer = new Timer();
    double val; 
        
    public AutoOpenGrippers (){
     addRequirements(RobotContainer.m_grp);
    }
    
    @Override
    public void initialize() {
     OpenGripTimer.reset();
     OpenGripTimer.start();
  }
    @Override
    public void execute() {
        val = OpenGripTimer.get();
        if (val < 2) {
         // new InstantCommand(()-> m_grp.openGrippers());
       // }
            RobotContainer.m_grp.openGrippers();}

  }
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return (val > 4);
  }
}
