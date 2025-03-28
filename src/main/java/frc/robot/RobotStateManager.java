package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotStateManager {
	public static class CoralManager {
		private static final DigitalInput frontCoral = Constants.FRONT_CORAL_SENSOR;
		private static final DigitalInput backCoral = Constants.BACK_CORAL_SENSOR;

		public static boolean hasCoral() {
			return frontCoral.get() || backCoral.get();
		}

		public static boolean elevatorBlocked() {
			return !Constants.frontSensor() && Constants.backSensor();
		}
	}
}
