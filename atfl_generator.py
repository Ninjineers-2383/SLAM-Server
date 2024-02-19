import numpy
from wpimath.geometry import Pose3d, Rotation3d, Quaternion, Transform3d, Translation3d

poses_optimized = numpy.genfromtxt("poses_optimized.txt",
                                   usecols=(0, 1, 2, 3, 4, 5, 6, 7))
tag_count = int(input('Number of tags: '))
tags_optimized = poses_optimized[:tag_count]

pin = 5

pin_loc = Pose3d(
    14.700757999999999,
    8.2042,
    1.355852,
    Rotation3d(Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476)),
)

pin_index = -1

for i in range(tag_count):
    if poses_optimized[i][0] == pin:
        pin_index = i
        break

if pin_index == -1:
    raise Exception('Could not find pin tag')

print(tags_optimized[pin_index])
print(pin_loc)

layout = []

pin_pose = Pose3d(
    tags_optimized[pin_index][1],
    tags_optimized[pin_index][2],
    tags_optimized[pin_index][3],
    Rotation3d(
        Quaternion(
            tags_optimized[pin_index][7],
            tags_optimized[pin_index][4],
            tags_optimized[pin_index][5],
            tags_optimized[pin_index][6],
        )
    ),
)

offset = pin_loc.relativeTo(pin_pose)

offset = Transform3d(offset.translation(), offset.rotation())

for i in range(tag_count):
    print(f"Tag {tags_optimized[i][0]} to {pin}:")
    layout.append(
        Pose3d(
            Translation3d(
                tags_optimized[i][1] + offset.X(),
                tags_optimized[i][2] + offset.Y(),
                tags_optimized[i][3] + offset.Z(),
            ),
            Rotation3d(
                Quaternion(
                    tags_optimized[i][7],
                    tags_optimized[i][4],
                    tags_optimized[i][5],
                    tags_optimized[i][6],
                )
            ),
        )
    )

print(layout)

tot_tags = int(input("Total number of tags: "))

tags_total = [i for i in range(1, tot_tags + 1)]
