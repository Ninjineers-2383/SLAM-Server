package com.team2383.SLAM.server.SLAM;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBuffer;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMBufferEntry;
import com.team2383.SLAM.server.SLAM.buffer.EKFSLAMState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * EKFSLAM is a Simultaneous Localization and Mapping algorithm that uses an
 * extended Kalman filter to estimate the
 * robot's pose and the pose of landmarks in the environment.
 * <p>
 * The state vector is a 7x1 matrix of the form [x, y, z, qw, qx, qy, qz] where
 * x, y, and z are the robot's position in
 * meters and qw, qx, qy, and qz are the components of the robot's quaternion
 * orientation.
 * The state vector is augmented with the pose of each landmark in the
 * environment.
 */
public class TimeSyncedEKFSLAM {
    // 7 x n matrix that maps the robot state to the full state vector.
    private final SimpleMatrix F_x;

    // Function that takes the robot's control input and the
    // current state vector and returns the predicted change in the state vector.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model;
    // Function that takes the robot's control input
    // and the current state vector and returns the Jacobian of the motion model
    // w.r.t. the state vector.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> motion_model_jacobian;
    // Function that takes the current state vector and a
    // landmark index and returns the predicted observation of the landmark.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model;
    // Function that takes the current state vector
    // and a landmark index and returns the Jacobian of the sensor model w.r.t. the
    // state vector of the robot and landmark.
    private final BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> sensor_model_jacobian;

    private final Pose3d[] initialLandmarkPoses;

    // Keeps track of which landmarks have been seen
    private final boolean[] seenLandmarks;

    private boolean enabled = false;

    private final EKFSLAMBuffer buffer;

    private final int numLandmarks;

    /**
     * Constructs an EKFSLAM object.
     *
     * @param numLandmarks
     *                     the number of landmarks in the environment
     */
    public TimeSyncedEKFSLAM(int numLandmarks, Pose3d[] initialLandmarkPoses) {
        this.buffer = new EKFSLAMBuffer(this::getInitialState);

        // Construct F_x matrix with 1s on the left block diagonal
        F_x = new SimpleMatrix(7, (numLandmarks + 1) * 7);
        for (int i = 0; i < 7; i++) {
            F_x.set(i, i, 1);
        }

        this.numLandmarks = numLandmarks;
        this.initialLandmarkPoses = initialLandmarkPoses;

        // Initialize motion model and sensor model
        motion_model = new MotionModel(F_x);
        motion_model_jacobian = new MotionModelJacobian(F_x);
        sensor_model = new SensorModelTag();
        sensor_model_jacobian = new SensorModelTagJacobian();

        seenLandmarks = new boolean[numLandmarks];
    }

    /**
     * Seeds the EKFSLAM algorithm with the initial pose of landmarks in the
     * environment. This should be called before the first call to predict() or
     * correct().
     * <p>
     * If a landmark is not seen, the corresponding pose should be null.
     * <p>
     * Useful for seeding the algorithm with the official field map at the start
     * of a match in order to get semi-accurate pose estimates before a map is
     * constructed.
     * 
     * @param landmarks
     *                  an array of poses of the landmarks in the environment
     */
    public SimpleMatrix seedLandmarks(SimpleMatrix mu, Pose3d[] landmarks) {
        for (int i = 0; i < landmarks.length; i++) {
            if (landmarks[i] != null) {
                mu.set(7 * (i + 1), 0, landmarks[i].getTranslation().getX());
                mu.set(7 * (i + 1) + 1, 0, landmarks[i].getTranslation().getY());
                mu.set(7 * (i + 1) + 2, 0, landmarks[i].getTranslation().getZ());
                mu.set(7 * (i + 1) + 3, 0, landmarks[i].getRotation().getQuaternion().getW());
                mu.set(7 * (i + 1) + 4, 0, landmarks[i].getRotation().getQuaternion().getX());
                mu.set(7 * (i + 1) + 5, 0, landmarks[i].getRotation().getQuaternion().getY());
                mu.set(7 * (i + 1) + 6, 0, landmarks[i].getRotation().getQuaternion().getZ());
                seenLandmarks[i] = true;
            }
        }

        return mu;
    }

    private SimpleMatrix setInitialRobotPose(SimpleMatrix mu, Pose3d pose) {
        mu.set(0, pose.getTranslation().getX());
        mu.set(1, pose.getTranslation().getY());
        mu.set(2, pose.getTranslation().getZ());
        mu.set(3, pose.getRotation().getQuaternion().getW());
        mu.set(4, pose.getRotation().getQuaternion().getX());
        mu.set(5, pose.getRotation().getQuaternion().getY());
        mu.set(6, pose.getRotation().getQuaternion().getZ());

        enabled = true;

        return mu;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public EKFSLAMState getInitialState(EKFSLAMBufferEntry entry) {
        Pose3d initialPose = initialLandmarkPoses[entry.visionResult.get().landmarkIndex()]
                .plus(entry.visionResult.get().robotToTag().inverse());

        return new EKFSLAMState(initialMu(initialPose), initialSigma());
    }

    private SimpleMatrix initialMu(Pose3d initialPose) {
        // Initialize empty state vector
        SimpleMatrix mu = new SimpleMatrix(7 * (numLandmarks + 1), 1);
        // Set 0 rotation quaternion
        for (int i = 0; i < 7 * (numLandmarks + 1); i += 7) {
            mu.set(i + 3, 0, 1);
        }

        mu = setInitialRobotPose(mu, initialPose);
        mu = seedLandmarks(mu, initialLandmarkPoses);
        return mu;
    }

    private SimpleMatrix initialSigma() {
        // Initialize covariance matrix
        SimpleMatrix sigma = new SimpleMatrix(7 * (seenLandmarks.length + 1), 7 * (seenLandmarks.length + 1));
        for (int i = 7; i < 7 * (seenLandmarks.length + 1); i++) {
            sigma.set(i, i, 0.5);
        }

        return sigma;
    }

    public void addDriveOdometryMeasurement(ChassisSpeeds speeds, double timestamp) {
        System.out.println("Chassis" + speeds.toString() + timestamp);
        // If the buffer is empty do not add the entry
        if (buffer.isEmpty()) {
            return;
        }

        // Add the entry
        buffer.addSpeedsEntry(new EKFSLAMBufferEntry(speeds, timestamp));

        // Try to update the chunk
        updateChunk();

    }

    public void addVisionMeasurement(Transform3d robotToTag, int landmarkIndex, double timestamp) {
        System.out.println("Vision");

        buffer.addVisionEntry(new EKFSLAMBufferEntry(robotToTag, landmarkIndex, timestamp));

        updateChunk();
    }

    public void updateChunk() {
        // If the buffer is empty do not update the chunk
        if (buffer.isEmpty()) {
            return;
        }

        // If the earliest unprocessed entry is not followed by a speeds entry, do not
        // update the chunk
        int earliestEmptyState = buffer.getEarliestEmptyState();

        int secondSpeedsIndex = buffer.getSecondSpeedsIndex();

        if (earliestEmptyState > secondSpeedsIndex) {
            return;
        }

        for (int i = 1; i <= secondSpeedsIndex; i++) {
            double deltaTime = buffer.get(i).timestamp - buffer.get(i - 1).timestamp;

            predict(buffer.interpolateSpeeds(i), deltaTime, buffer.get(i - 1).state.get().mu(),
                    buffer.get(i - 1).state.get().sigma(),
                    i);
            if (buffer.get(i).isVisionEntry()) {
                correct(buffer.get(i).visionResult.get().robotToTag(), buffer.get(i).visionResult.get().landmarkIndex(),
                        buffer.get(i).state.get().mu(),
                        buffer.get(i).state.get().sigma(), i);
            }
        }

        System.out.println("Completed Chunk");

    }

    /**
     * Extended Kalman filter prediction step.
     * <p>
     * Updates the state vector and covariance matrix based on the robot's
     * control input.
     * 
     * @param speeds
     *               the robot's control input
     * @param dt
     *               the time elapsed since the last call to predict()
     * @return the predicted pose of the robot
     */
    public Pose3d predict(ChassisSpeeds speeds, double dt, SimpleMatrix previousMu, SimpleMatrix previousSigma,
            int index) {
        SimpleMatrix u = new SimpleMatrix(3, 1, true,
                speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, speeds.omegaRadiansPerSecond * dt);
        SimpleMatrix R = SimpleMatrix.diag(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        SimpleMatrix muHat = previousMu.plus(motion_model.apply(u, previousMu));
        SimpleMatrix G = SimpleMatrix.identity((numLandmarks + 1) * 7).plus(motion_model_jacobian.apply(u, previousMu));
        SimpleMatrix sigmaHat = G.mult(previousSigma.mult(G.transpose())).plus(F_x.transpose().mult(R.mult(F_x)));

        buffer.get(index).updateMuAndSigma(muHat, sigmaHat);

        return getPose(muHat, 0);
    }

    /**
     * Extended Kalman filter correction step.
     * <p>
     * Updates the state vector and covariance matrix based on the robot's
     * observation of a landmark.
     * 
     * @param robotToTag
     *                      the robot's observation of the landmark
     * @param landmarkIndex
     *                      the index of the landmark in the state vector
     * @return the corrected pose of the robot
     */
    public Pose3d correct(Transform3d robotToTag, int landmarkIndex, SimpleMatrix muHat, SimpleMatrix sigmaHat,
            int index) {
        if (!enabled) {
            return getRobotPose();
        }
        SimpleMatrix z_obs = new SimpleMatrix(7, 1, true,
                robotToTag.getTranslation().getX(), robotToTag.getTranslation().getY(),
                robotToTag.getTranslation().getZ(),
                robotToTag.getRotation().getQuaternion().getW(),
                robotToTag.getRotation().getQuaternion().getX(),
                robotToTag.getRotation().getQuaternion().getY(),
                robotToTag.getRotation().getQuaternion().getZ());

        if (!seenLandmarks[landmarkIndex]) {
            seenLandmarks[landmarkIndex] = true;
            Pose3d tag = getRobotPose().plus(robotToTag);
            muHat.set(7 * (landmarkIndex + 1), 0, tag.getTranslation().getX());
            muHat.set(7 * (landmarkIndex + 1) + 1, 0, tag.getTranslation().getY());
            muHat.set(7 * (landmarkIndex + 1) + 2, 0, tag.getTranslation().getZ());
            muHat.set(7 * (landmarkIndex + 1) + 3, 0, tag.getRotation().getQuaternion().getW());
            muHat.set(7 * (landmarkIndex + 1) + 4, 0, tag.getRotation().getQuaternion().getX());
            muHat.set(7 * (landmarkIndex + 1) + 5, 0, tag.getRotation().getQuaternion().getY());
            muHat.set(7 * (landmarkIndex + 1) + 6, 0, tag.getRotation().getQuaternion().getZ());
        }

        SimpleMatrix z_pred = sensor_model.apply(muHat, new SimpleMatrix(1, 1, true,
                landmarkIndex));

        SimpleMatrix H = sensor_model_jacobian.apply(muHat, new SimpleMatrix(1, 1, true,
                landmarkIndex));

        SimpleMatrix F_xj = new SimpleMatrix(14, muHat.getNumRows());
        for (int i = 0; i < 7; i++) {
            F_xj.set(i, i, 1);
            F_xj.set(i + 7, i + 7 * (landmarkIndex + 1), 1);
        }

        SimpleMatrix Hi = H.mult(F_xj);

        SimpleMatrix Q = SimpleMatrix.diag(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

        SimpleMatrix S = Hi.mult(sigmaHat.mult(Hi.transpose())).plus(Q).invert();

        SimpleMatrix K = sigmaHat.mult(Hi.transpose()).mult(S);

        SimpleMatrix mu = muHat.plus(K.mult(subtractPose(z_obs, z_pred)));
        SimpleMatrix sigma = (SimpleMatrix.identity(K.getNumRows()).minus(K.mult(Hi))).mult(sigmaHat);

        buffer.get(index).updateMuAndSigma(mu, sigma);

        return getPose(mu, 0);
    }

    /**
     * Returns the pose of the robot.
     * 
     * @return the pose of the robot
     */
    public Pose3d getRobotPose() {
        return getPose(getLatestMu(), 0);
    }

    /**
     * Returns the pose of a landmark.
     * 
     * @param landmarkIndex
     *                      the index of the landmark in the state vector
     * @return the pose of the landmark
     */
    public Pose3d getLandmarkPose(int landmarkIndex) {
        return getPose(getLatestMu(), 7 * (landmarkIndex + 1));
    }

    /**
     * Returns the poses of all landmarks.
     * 
     * @return the poses of all landmarks
     */
    public Pose3d[] getLandmarkPoses() {
        if (getLatestMu() == null) {
            return null;
        }
        Pose3d[] poses = new Pose3d[(getLatestMu().getNumRows() - 7) / 7];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = getPose(getLatestMu(), 7 * (i + 1));
        }
        return poses;
    }

    public EKFSLAMState getLatestState() {
        if (buffer.getEarliestEmptyState() == 0) {
            return new EKFSLAMState(null, null);
        } else {
            return buffer.get(buffer.getEarliestEmptyState() - 1).state.get();
        }
    }

    public SimpleMatrix getLatestMu() {
        return getLatestState().mu();
    }

    /**
     * Returns the covariance matrix
     * 
     * @return the covariance matrix: sigma
     */
    public SimpleMatrix getLatestSigma() {
        return getLatestState().sigma();
    }

    private SimpleMatrix subtractPose(SimpleMatrix A, SimpleMatrix B) {
        if (Math.signum(A.get(6)) != Math.signum(B.get(6))) {
            A.set(3, 0, -A.get(3));
            A.set(4, 0, -A.get(4));
            A.set(5, 0, -A.get(5));
            A.set(6, 0, -A.get(6));
        }

        return A.minus(B);
    }

    private Pose3d getPose(SimpleMatrix mu, int start) {
        if (mu == null) {
            return null;
        }
        return new Pose3d(mu.get(0 + start), mu.get(1 + start), mu.get(2 + start),
                new Rotation3d(
                        new Quaternion(mu.get(3 + start), mu.get(4 + start), mu.get(5 + start), mu.get(6 + start))));
    }
}
