package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SharedSpaceManager {
	private static SubsystemBase currentOwner = null;

	public static boolean requestAccess(SubsystemBase subsystem) {
		if (currentOwner == null || currentOwner == subsystem) {
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
