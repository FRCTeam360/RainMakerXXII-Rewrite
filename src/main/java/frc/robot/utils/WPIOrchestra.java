// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.util.Collection;

/** Add your docs here. */
public class WPIOrchestra extends Orchestra {
    public  WPIOrchestra(Collection<WPI_TalonFX> instruments) {
        super();

        for(WPI_TalonFX instrument : instruments) {
            addWPIInstrument(instrument);
        }
    }

    public ErrorCode addWPIInstrument(WPI_TalonFX instrument){
        int retval = Orchestra.JNI_AddInstrument(m_handle, instrument.getHandle());
        return ErrorCode.valueOf(retval);
    }
}
