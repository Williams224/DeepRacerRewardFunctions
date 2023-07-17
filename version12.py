import math

outer_track = {
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    48,
    49,
    50,
    51,
    52,
    53,
    54,
    55,
    56,
    57,
    58,
    59,
}
inner_track = {
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    33,
    34,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    67,
    68,
    69,
    70,
}
middle_track = {44, 45, 46, 47, 60, 61, 62, 63, 64, 65, 66}


def get_path_reward(
    next_waypoint,
    outer_track_waypoints,
    middle_track_waypoints,
    inner_track_waypoints,
    track_width,
    distance_from_center,
    is_left_of_center,
):
    if next_waypoint in middle_track_waypoints:
        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        # Give higher reward if the car is closer to center line and vice versa
        if distance_from_center <= marker_1:
            return 2.0
        elif distance_from_center <= marker_2:
            return 0.5
        elif distance_from_center <= marker_3:
            return 0.1
        else:
            return 1e-3
    elif next_waypoint in outer_track_waypoints:
        marker_1 = 0.33 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.1 * track_width

        if distance_from_center > marker_1 and not is_left_of_center:
            return 2.0
        elif distance_from_center > marker_2 and not is_left_of_center:
            return 0.5
        elif distance_from_center > marker_3 and not is_left_of_center:
            return 0.1
        else:
            return 1e-3

    elif next_waypoint in inner_track_waypoints or next_waypoint > 70:
        marker_1 = 0.33 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.1 * track_width
        if distance_from_center > marker_1 and is_left_of_center:
            return 2.0
        elif distance_from_center > marker_2 and is_left_of_center:
            return 0.5
        elif distance_from_center > marker_3 and is_left_of_center:
            return 0.1
        else:
            return 1e-3

    else:
        raise ValueError("crap")


class PARAMS:
    prev_speed = None
    prev_steering_angle = None
    prev_steps = None


def reward_function(params):
    steps = params["steps"]
    next_waypoint_i = params["closest_waypoints"][1]
    last_waypoint_i = params["closest_waypoints"][0]

    next_waypoint = params["waypoints"][next_waypoint_i]
    last_waypoint = params["waypoints"][last_waypoint_i]

    speed = params["speed"]

    abs_steering_angle = abs(params["steering_angle"])

    if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
        PARAMS.prev_speed = None
        PARAMS.prev_steering_angle = None
        PARAMS.prev_steps = None

    # spoonfeed
    if params["all_wheels_on_track"] == False:
        return float(1e-3)

    # don't go slow on first straight
    if speed < 2.9 and next_waypoint_i < 18:
        return float(1e-3)

    path_reward = get_path_reward(
        next_waypoint_i,
        outer_track,
        middle_track,
        inner_track,
        params["track_width"],
        params["distance_from_center"],
        params["is_left_of_center"],
    )

    track_dir = math.atan2(
        next_waypoint[1] - last_waypoint[1], next_waypoint[0] - last_waypoint[0]
    )
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

    if abs_steering_angle < 5.0:
        steering_reward = 2.0
    elif abs_steering_angle < 15.0:
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

    speed_reward = (params["speed"] ** 2) / 8.0

    PARAMS.prev_speed = params["speed"]
    PARAMS.prev_steering_angle = params["steering_angle"]
    PARAMS.prev_steps = params["steps"]

    path_weight = 1.5
    heading_weight = 1.0
    steering_weight = 0.5
    speed_weight = 0.25
    # slowdown_weight = 0.25
    steering_change_weight = 0.25
    progress_weight = 0.75

    total_reward = (
        path_weight * path_reward
        + heading_weight * heading_reward
        + steering_weight * steering_reward
        + speed_weight * speed_reward
        + steering_change_weight * steering_penalty
        + progress_weight * progress_reward
    )

    return float(total_reward)
