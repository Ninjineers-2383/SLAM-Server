package com.team2383.SLAM.server.helpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Kinematics.ProtobufChassisSpeeds;
import edu.wpi.first.util.protobuf.Protobuf;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class ChassisSpeedsProto implements Protobuf<ChassisSpeeds, ProtobufChassisSpeeds> {

    @Override
    public Class<ChassisSpeeds> getTypeClass() {
        return ChassisSpeeds.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufChassisSpeeds.getDescriptor();
    }

    @Override
    public ProtobufChassisSpeeds createMessage() {
        return ProtobufChassisSpeeds.newInstance();
    }

    @Override
    public ChassisSpeeds unpack(ProtobufChassisSpeeds msg) {
        return new ChassisSpeeds(msg.getVx(), msg.getVy(), msg.getOmega());
    }

    @Override
    public void pack(ProtobufChassisSpeeds msg, ChassisSpeeds value) {
        msg.setVx(value.vxMetersPerSecond);
        msg.setVy(value.vyMetersPerSecond);
        msg.setOmega(value.omegaRadiansPerSecond);
    }
    
}
