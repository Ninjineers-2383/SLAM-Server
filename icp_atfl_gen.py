from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Quaternion, Transform3d
import numpy as np
import json

from atfl_json import exportLayout

poses_optimized = np.genfromtxt("poses_optimized.txt", usecols=(0, 1, 2, 3, 4, 5, 6, 7))
tag_count = int(input('Number of tags: '))
tags_optimized = poses_optimized[:tag_count]

loaded_ids = [int(tag[0]) for tag in tags_optimized]

official_poses = None
with open("2024-crescendo.json", "r") as f:
    official_poses = json.load(f)

tags_official = [[
    tag['ID'],
    tag['pose']['translation']['x'],
    tag['pose']['translation']['y'],
    tag['pose']['translation']['z'],
    tag['pose']['rotation']['quaternion']['X'],
    tag['pose']['rotation']['quaternion']['Y'],
    tag['pose']['rotation']['quaternion']['Z'],
    tag['pose']['rotation']['quaternion']['W'],
    ] for tag in official_poses['tags'] if tag['ID'] in loaded_ids]


p1_t = np.array([tag[1:4] for tag in tags_optimized])
p2_t = np.array([tag[1:4] for tag in tags_official])


p1 = p1_t.transpose()
p2 = p2_t.transpose()

p1_c = np.mean(p1, axis=1).reshape((-1, 1))
p2_c = np.mean(p2, axis=1).reshape((-1, 1))

q1 = p1 - p1_c
q2 = p2 - p2_c

H = np.matmul(q1, q2.transpose())

U, X, V_t = np.linalg.svd(H)

R = np.matmul(V_t.transpose(), U.transpose())

assert np.allclose(np.linalg.det(R), 1.0)

T = p2_c - np.matmul(R, p1_c)

result = T + np.matmul(R, p1)

rot = Rotation3d(R)

trans = Transform3d(T[0][0], T[1][0], T[2][0], rot)

poses = [
        Pose3d(
            Translation3d(tag[1], tag[2], tag[3]),
            Rotation3d(Quaternion(tag[7], tag[4], tag[5], tag[6])))
        for tag in tags_optimized]

poses_rotated = [
    pose.rotateBy(rot)
    # pose
    for pose in poses]

poses_transformed = [
    Pose3d(
        pose.X() + trans.X(),
        pose.Y() + trans.Y(),
        pose.Z() + trans.Z(),
        pose.rotation()
    )
    for pose in poses_rotated
]

with open('out.json', 'w') as f:
    js = exportLayout(poses_transformed, loaded_ids, 16)
    json.dump(js, f)
