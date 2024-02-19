def exportLayout(layout, ids, tag_count):
    tags_total = [i + 1 for i in range(tag_count)]
    json_data = {"tags": [], "field": {"length": 0.0, "width": 0.0}}
    i = 0
    for tag in layout:
        # Unpack row elements
        id = ids[i]
        tags_total.remove(id)

        i += 1

        # Turn yaw into quaternion
        q = tag.rotation().getQuaternion()

        json_data["tags"].append(
            {
                "ID": int(id),
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

    for id in tags_total:
        json_data["tags"].append(
            {
                "ID": int(id),
                "pose": {
                    "translation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                    },
                    "rotation": {
                        "quaternion": {
                            "W": 1.0,
                            "X": 0.0,
                            "Y": 0.0,
                            "Z": 0.0,
                        }
                    },
                },
            }
        )

    return json_data
