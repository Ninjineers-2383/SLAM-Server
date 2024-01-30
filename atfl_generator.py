import json
import numpy
from wpimath.geometry import Pose3d, Rotation3d, Quaternion, Transform3d, Translation3d
from wpimath import units

poses_optimized = numpy.genfromtxt("poses_optimized.txt", usecols=(1, 2, 3, 4, 5, 6, 7))
tags_optimized = poses_optimized[:16]

pin = 0

pin_loc = Pose3d(
    units.inchesToMeters(593.68),
    units.inchesToMeters(9.68),
    units.inchesToMeters(53.38),
    Rotation3d(0, 0, units.degreesToRadians(120)),
)

print(tags_optimized[pin])
print(pin_loc)

layout = []

pin_pose = Pose3d(
    tags_optimized[pin][0],
    tags_optimized[pin][1],
    tags_optimized[pin][2],
    Rotation3d(
        Quaternion(
            tags_optimized[pin][6],
            tags_optimized[pin][3],
            tags_optimized[pin][4],
            tags_optimized[pin][5],
        )
    ),
)

offset = pin_loc.relativeTo(pin_pose)

offset = Transform3d(offset.translation(), offset.rotation())

for i in range(16):
    print(f"Tag {i} to {pin}:")
    layout.append(
        Pose3d(
            Translation3d(
                tags_optimized[i][0] + offset.X(),
                tags_optimized[i][1] + offset.Y(),
                tags_optimized[i][2] + offset.Z(),
            ),
            Rotation3d(
                Quaternion(
                    tags_optimized[i][6],
                    tags_optimized[i][3],
                    tags_optimized[i][4],
                    tags_optimized[i][5],
                )
            ),
        )
    )

print(layout)


def exportLayout(layout):
    json_data = {"tags": [], "field": {"length": 0.0, "width": 0.0}}
    i = 1
    for tag in layout:
        # Unpack row elements
        id = i

        i += 1

        # Turn yaw into quaternion
        q = tag.rotation().getQuaternion()

        json_data["tags"].append(
            {
                "ID": id,
                "pose": {
                    "translation": {
                        "x": tag.X(),
                        "y": tag.Y(),
                        "z": tag.Z(),
                    },
                    "rotation": {
                        "quaternion": {
                            "W": q.W(),
                            "X": q.X(),
                            "Y": q.Y(),
                            "Z": q.Z(),
                        }
                    },
                },
            }
        )

    # Write JSON
    with open("out.json", "w") as f:
        json.dump(json_data, f, indent=2)


exportLayout(layout)
