package com.team2383.SLAM.server.helpers;

import java.nio.ByteBuffer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;

public class ChassisSpeedsStruct implements Struct<ChassisSpeeds> {

    @Override
    public Class<ChassisSpeeds> getTypeClass() {
        return ChassisSpeeds.class;
    }

    @Override
    public String getTypeString() {
        return "struct:ChassisSpeeds";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 3;
    }

  
    @Override
    public String getSchema() {
      return "double vxMetersPerSecond;double vyMetersPerSecond;double omegaRadiansPerSecond";
    }

    @Override
    public ChassisSpeeds unpack(ByteBuffer bb) {
        double vxMetersPerSecond = bb.getDouble();
        double vyMetersPerSecond = bb.getDouble();
        double omegaRadiansPerSecond = bb.getDouble();

        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    @Override
    public void pack(ByteBuffer bb, ChassisSpeeds value) {
        bb.putDouble(value.vxMetersPerSecond);
        bb.putDouble(value.vyMetersPerSecond);
        bb.putDouble(value.omegaRadiansPerSecond);
    }
}
