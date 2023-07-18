import math
from math import e, sqrt, pi

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


def gaussian(x, m, s):
    gauss = 1 / (sqrt(2 * pi) * s) * e ** (-0.5 * (float(x - m) / s) ** 2)
    return gauss


def get_path_reward_two(
    nearest_way_point,
    out_points,
    center_points,
    in_points,
    is_left_of_center,
    distance_from_center_w,
):
    if is_left_of_center:
        distance_from_center_unabs = distance_from_center_w * -1
    else:
        distance_from_center_unabs = distance_from_center_w

    if nearest_way_point in center_points:
        return gaussian(distance_from_center_unabs, 0.0, 0.15) * 0.75
    elif nearest_way_point in out_points:
        return gaussian(distance_from_center_unabs, 0.2, 0.15) * 0.75
    elif nearest_way_point in in_points or nearest_way_point > 70:
        return gaussian(distance_from_center_unabs, -0.2, 0.15) * 0.75


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
    # if speed < 2.9 and next_waypoint_i < 18:
    # return float(1e-3)

    distance_from_center_w = params["distance_from_center"] / params["track_width"]
    path_reward = get_path_reward_two(
        next_waypoint_i,
        outer_track,
        middle_track,
        inner_track,
        distance_from_center_w,
        params["is_left_of_center"],
    )

    track_dir = math.atan2(
        next_waypoint[1] - last_waypoint[1], next_waypoint[0] - last_waypoint[0]
    )
    track_degrees = math.degrees(track_dir)
    direction_diff = abs(track_degrees - params["heading"])

    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    heading_reward = gaussian(direction_diff, 0.0, 10.0) * 50.0

    steering_reward = gaussian(abs_steering_angle, 0.0, 20.0) * 100.0

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

    speed_reward = gaussian(speed, 4.0, 1.75) * 8.75

    PARAMS.prev_speed = params["speed"]
    PARAMS.prev_steering_angle = params["steering_angle"]
    PARAMS.prev_steps = params["steps"]

    path_weight = 2.0
    heading_weight = 1.0
    steering_weight = 0.5
    speed_weight = 0.25
    # slowdown_weight = 0.25
    steering_change_weight = 0.25
    progress_weight = 0.25

    total_reward = (
        path_weight * path_reward
        + heading_weight * heading_reward
        + steering_weight * steering_reward
        + speed_weight * speed_reward
        + steering_change_weight * steering_penalty
        + progress_weight * progress_reward
    )

    return float(total_reward)
