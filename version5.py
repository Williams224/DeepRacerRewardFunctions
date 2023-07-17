import math


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
    speed_weight = 0.5
    slowdown_weight = 0.25
    steering_change_weight = 0.25

    # Read input parameters
    track_width = params["track_width"]
    distance_from_center = params["distance_from_center"]

    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        path_reward = 2.0
    elif distance_from_center <= marker_2:
        path_reward = 0.5
    elif distance_from_center <= marker_3:
        path_reward = 0.1
    else:
        path_reward = 1e-3  # likely crashed/ close to off track

    # heading weight, give much higher weights pointing to the future direction of the track
    waypoints = params["waypoints"]
    next_point = waypoints[params["closest_waypoints"][1]]
    prev_point = waypoints[params["closest_waypoints"][0]]
    track_dir = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_degrees = math.degrees(track_dir)
    direction_diff = abs(track_degrees - params["heading"])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if direction_diff < 10.0:
        heading_reward = 2.0
    elif direction_diff > 10.0 and direction_diff <= 30.0:
        heading_reward = 1.0
    elif direction_diff > 30.0 and direction_diff <= 45.0:
        heading_reward = 0.45
    else:
        heading_reward = 1e-3

    # steering_reward, less steering the better
    if params["heading"] < 5.0:
        steering_reward = 2.0
    elif params["heading"] < 15.0:
        steering_reward = 1.0
    else:
        steering_reward = 0.5

    slow_down_penalty = 0.0
    if PARAMS.prev_speed is not None:
        slowed_down = PARAMS.prev_speed > params["speed"]
        corner_coming_up = track_degrees > 10.0
        if slowed_down and not corner_coming_up:
            slow_down_penalty = -2.0

    steering_penalty = 0.0
    if PARAMS.prev_steering_angle is not None:
        steering_changed = (
            abs(PARAMS.prev_steering_angle - params["steering_angle"]) - 1.0
        ) > 0.0
        if steering_changed:
            steering_penalty = -2.0

    reward = (
        path_weight * path_reward
        + heading_weight * heading_reward
        + steering_weight * steering_reward
        + slowdown_weight * slow_down_penalty
        + steering_penalty * steering_change_weight
    )

    PARAMS.prev_speed = params["speed"]
    PARAMS.prev_steering_angle = params["steering_angle"]
    PARAMS.prev_steps = params["steps"]

    return float(reward)
