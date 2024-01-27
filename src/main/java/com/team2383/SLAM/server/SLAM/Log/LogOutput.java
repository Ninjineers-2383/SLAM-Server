package com.team2383.SLAM.server.SLAM.Log;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class LogOutput {
    private final HashMap<Integer, LogVertex> vertices;

    private final ArrayList<LogEdge> edges;

    private int lastID = 100;

    public LogOutput() {
        this.vertices = new HashMap<>();
        this.edges = new ArrayList<>();
    }

    public int addChassisState(Pose3d pose, SimpleMatrix information) {
        int prevId = lastID;
        lastID += 1;
        int id = lastID;
        vertices.put(id, new LogVertex(id, pose));

        if (prevId != 100) {
            edges.add(new LogEdge(prevId, id, pose.minus(vertices.get(prevId).pose()),
                    information));
        }

        return id;
    }

    public void addVisionEdge(int from, int to, Transform3d vision, SimpleMatrix information) {
        if (!vertices.containsKey(to)) {
            vertices.put(to, new LogVertex(to, vertices.get(from).pose().plus(vision)));
        }
        edges.add(new LogEdge(from, to, vision, information));
    }

    public Pose3d getLatestPose() {
        if (lastID == 100) {
            return new Pose3d();
        }
        return vertices.get(lastID).pose();
    }

    public void saveg2o(String filename) {
        File file = new File(filename);
        System.out.println(filename);
        try {
            file.createNewFile();

            FileWriter writer = new FileWriter(file);

            StringBuilder builder = new StringBuilder();

            for (LogVertex vertex : vertices.values()) {
                builder.append(String.format("VERTEX_SE3:QUAT %d %f %f %f %f %f %f %f\n", vertex.id(),
                        vertex.pose().getTranslation().getX(), vertex.pose().getTranslation().getY(),
                        vertex.pose().getTranslation().getZ(),
                        vertex.pose().getRotation().getQuaternion().getX(),
                        vertex.pose().getRotation().getQuaternion().getY(),
                        vertex.pose().getRotation().getQuaternion().getZ(),
                        vertex.pose().getRotation().getQuaternion().getW()));
            }

            for (LogEdge edge : edges) {
                // EDGE_SE3:QUAT
                builder.append(String.format("EDGE_SE3:QUAT %d %d %f %f %f %f %f %f %f " +
                        "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                        edge.from(), edge.to(),
                        edge.distance().getTranslation().getX(), edge.distance().getTranslation().getY(),
                        edge.distance().getTranslation().getZ(),
                        edge.distance().getRotation().getQuaternion().getX(),
                        edge.distance().getRotation().getQuaternion().getY(),
                        edge.distance().getRotation().getQuaternion().getZ(),
                        edge.distance().getRotation().getQuaternion().getW(),
                        edge.information().get(0, 0), edge.information().get(0, 1), edge.information().get(0, 2),
                        edge.information().get(0, 3), edge.information().get(0, 4), edge.information().get(0, 5),
                        edge.information().get(1, 1), edge.information().get(1, 2), edge.information().get(1, 3),
                        edge.information().get(1, 4), edge.information().get(2, 5), edge.information().get(2, 2),
                        edge.information().get(2, 3), edge.information().get(2, 4), edge.information().get(2, 5),
                        edge.information().get(3, 3), edge.information().get(3, 4), edge.information().get(3, 5),
                        edge.information().get(4, 4), edge.information().get(4, 5), edge.information().get(5, 5)));
            }

            writer.write(builder.toString());

            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
