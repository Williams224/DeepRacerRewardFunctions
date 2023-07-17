import math

# zigzagging
# offtrack
# racing line?


class PARAMS:
    prev_speed = None
    prev_steering_angle = None
    prev_steps = None


def reward_function(params):
    """
    Example of rewarding the agent to follow center line
    """

    steps = params["steps"]

    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None
        PARAMS.prev_steps = None
        PARAMS.prev_direction_diff = None

    if params["all_wheels_on_track"] == False:
        return float(1e-3)

    path_weight = 1.0
    heading_weight = 1.0
    steering_weight = 0.5
    speed_weight = 0.1
    slowdown_weight = 0.25
    steering_change_weight = 0.25
    progress_weight = 0.75

    # Read input parameters
    track_width = params["track_width"]
    distance_from_center = params["distance_from_center"]

    waypoints = params["waypoints"]
    if params["closest_waypoints"][1] + 1 < len(waypoints):
        next_point = waypoints[params["closest_waypoints"][1] + 1]
    else:
        next_point = waypoints[-1]

    prev_point = waypoints[params["closest_waypoints"][0]]

    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # keep to inside during hairpin otherwise stay close to center
    closest_next_wp_index = params["closest_waypoints"][1]
    if closest_next_wp_index > 15 and closest_next_wp_index < 41:
        if distance_from_center > marker_2 and params["is_left_of_center"]:
            path_reward = 2.0
        elif distance_from_center > marker_1 and params["is_left_of_center"]:
            path_reward = 1.0
        elif params["is_left_of_center"]:
            path_reward = 0.5
        else:
            return 1e-3

    else:
        if distance_from_center <= marker_1:
            path_reward = 2.0
        elif distance_from_center <= marker_2:
            path_reward = 0.5
        elif distance_from_center <= marker_3:
            path_reward = 0.1
        else:
            return 1e-3

    # heading weight, give much higher weights pointing to the future direction of the track

    # first straight
    if params["closest_waypoints"][1] < 15 and params["steering_angle"] > 2.0:
        return 1e-3

    # hairpin specific
    if params["closest_waypoints"][1] >= 32 and params["closest_waypoints"][1] < 40:
        if params["steering_angle"] < 0.0:
            return 1e-3

        path_weight = path_weight * 5

    track_dir = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_degrees = math.degrees(track_dir)
    direction_diff = abs(track_degrees - params["heading"])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if direction_diff < 10.0:
        heading_reward = 2.0
    elif direction_diff > 10.0 and direction_diff <= 30.0:
        heading_reward = 1.0
    else:
        heading_reward = 1e-3

    # steering_reward, less steering the better
    if abs(params["steering_angle"]) < 10.0:
        steering_reward = 2.0
    elif abs(params["steering_angle"]) < 20.0:
        steering_reward = 1.0
    else:
        steering_reward = 0.5

    steering_penalty = 0.0
    if PARAMS.prev_steering_angle is not None:
        steering_changed = (
            abs(PARAMS.prev_steering_angle - params["steering_angle"]) - 1.0
        ) > 0.0
        if steering_changed:
            steering_penalty = -2.0

    progress_reward = 0
    if params["progress"] % 10 == 0 and params["progress"] >= 10:
        progress_reward = int(params["progress"] / 10) * 10

    if params["speed"] > 3.9:
        speed_reward = 2.0
    elif params["speed"] > 2.5:
        speed_reward = 1.0
    else:
        speed_reward = 0.0

    reward = (
        path_weight * path_reward
        + heading_weight * heading_reward
        + steering_weight * steering_reward
        + steering_penalty * steering_change_weight
        + progress_reward * progress_weight
        + speed_reward * speed_weight
    )

    PARAMS.prev_speed = params["speed"]
    PARAMS.prev_steering_angle = params["steering_angle"]
    PARAMS.prev_steps = params["steps"]

    return float(reward)
