file_path = "/home/acca/waypoints.txt"

def waypoint(wpt_name, line_number, line):
    if line.strip() == wpt_name:
        # Extract frame_id and pose's value
        frame_id_line = lines[line_number + 5].strip()
        pose_lines = lines[line_number + 6:line_number + 16]

        # Extract specific values from frame_id_line
        frame_id_value = frame_id_line.split(":")[-1].strip()

        # Extract specific values from pose_lines
        position_values = [line.split(":")[-1].strip() for line in pose_lines[2:5]]
        orientation_values = [line.split(":")[-1].strip() for line in pose_lines[6:10]]

        # Print the extracted values
        print("Frame ID:", frame_id_value)
        print("Position (x, y, z):", position_values)
        print("Orientation (x, y, z, w):", orientation_values)

        return (position_values, orientation_values)

    else:
        print(f"{'[wpt2]'} not found")
        return False

with open(file_path, "r", encoding="UTF-8") as f:
    lines = f.readlines()
    lane_id = int(input('lane_id: '))

    for line_number, line in enumerate(lines):
        if lane_id == 192:
            res = waypoint("[wpt2]",line_number=line_number,line=line)
            if res != False:
                pose = dkjfsd
                break
            else:
                pass
