package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateManager {
	public static class ElevatorAGIManager {
		private static SubsystemBase currentOwner = null;

		public static boolean requestAccess(SubsystemBase subsystem) {
			if(currentOwner == null || currentOwner == subsystem) {
				currentOwner = subsystem;
				return true;
			}
			return false;
		}

		public static void releaseAccess(SubsystemBase subsystem) {
			currentOwner = currentOwner == subsystem ? null : currentOwner;
		}

		public static String printOwner() {
			return currentOwner == null ? "None" : currentOwner.getClass().getName();
		}
	}

	public static class CoralManager {
		private static final DigitalInput frontCoral = Constants.FRONT_CORAL_SENSOR;
		private static final DigitalInput backCoral = Constants.BACK_CORAL_SENSOR;

		public static boolean hasCoral() {
			return frontCoral.get() || backCoral.get();
		}

		public static boolean elevatorBlocked() {
			return !frontCoral.get() && backCoral.get();
		}
	}
}
