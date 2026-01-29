package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotMath {
   public static double turnToFieldPoint(double robotX, double robotY, double targetX, double targetY) {
      double deltaX = targetX - robotX;
      double deltaY = targetY - robotY;
      return Math.toDegrees(Math.atan2(deltaY, deltaX));
   }

}
