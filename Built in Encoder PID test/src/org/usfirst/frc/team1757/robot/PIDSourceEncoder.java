package org.usfirst.frc.team1757.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceEncoder implements PIDSource{
		
	private Encoder enc;
	private CANTalon talon;
	
	PIDSourceEncoder(CANTalon cantalon) {
		talon = cantalon;
		enc = null;
	}
	
	PIDSourceEncoder(Encoder encoder) {
		enc = encoder;
		talon = null;
	}

	public void setPIDSourceType(PIDSourceType pidsource) {
		if (!(talon== null))
			talon.setPIDSourceType(pidsource);
		if (!(enc == null))
			enc.setPIDSourceType(pidsource);
	}

	public PIDSourceType getPIDSourceType() {
		if (!(talon== null))
			return talon.getPIDSourceType();
		if (!(enc == null))
			return enc.getPIDSourceType();
		else
			return null;
	}

	public double pidGet() {
		if (!(talon == null))
			return talon.getEncPosition();
		if (!(enc == null))
			return enc.getRaw();
		else
			return 0;
	}

}
