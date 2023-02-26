package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A class that manages simulation stuff
 */
public class Simulation {
  public static Field2d field = new Field2d();

  public static void updateSimulation(){
    SmartDashboard.putData("Field", field);
  }
}
