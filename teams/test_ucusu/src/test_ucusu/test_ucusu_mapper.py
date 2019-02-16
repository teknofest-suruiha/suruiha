from math import sqrt, pow


def distance(first, second):
    return sqrt(pow((second[0] - first[0]), 2) + pow((second[1] - first[1]), 2))


def way_point_generator(mapper_area_size, mapper_left_bottom, mapper_right_top,
                        uav_batteries, num_uavs, cruise_altitude):
    # >>> START OF CONFIG <<<

    # Will be given to us in competition
    runway_left_bottom = mapper_left_bottom
    runway_right_top = mapper_right_top

    # Will be given to us in competition
    # area_width = 600
    # area_height = 800
    area_width = mapper_area_size[0]
    area_height = mapper_area_size[1]

    # Will be given to us in competition
    uav_batteries = uav_batteries  # (1000, 999, 998, 997, 996, 995)
    battery_capacities = uav_batteries[:]

    # Our configs or given in competition
    numbers_uav = num_uavs

    # Our configs
    row_padding = 40  # 40
    column_padding = (cruise_altitude - 10) / 2  # 40

    # Guard bands for the big search area
    right_guard_band = 20
    left_guard_band = 20
    top_guard_band = 20
    bottom_guard_band = 20

    # Guard bands for small areas of UAVs
    area_right_guard_band = 0
    area_left_guard_band = 0
    area_top_guard_band = 0
    area_bottom_guard_band = 0

    # Search area will be bigger than its actual size
    height_guard_band = 0
    area_width_guard_band = 0

    # Battery configs
    battery_usage_per_meter = 0
    repeat = 2
    guard_coefficient = 1.2  # >= 1.0 and 1.5 is ideal

    # >>> END OF CONFIG <<<

    runway = [
        runway_left_bottom,
        (runway_left_bottom[0], runway_right_top[1]),
        runway_right_top,
        (runway_right_top[0], runway_left_bottom[1]),
    ]

    if area_width >= area_height:
        search_area = [
            (-(area_width / 2 + area_width_guard_band / 2), -(area_height / 2 + height_guard_band / 2)),
            (-(area_width / 2 + area_width_guard_band / 2), area_height / 2 + height_guard_band / 2),
            (area_width / 2 + area_width_guard_band / 2, area_height / 2 + height_guard_band / 2),
            (area_width / 2 + area_width_guard_band / 2, -(area_height / 2 + height_guard_band / 2)),
        ]
    else:
        search_area = [
            (area_width / 2 + area_width_guard_band / 2, -(area_height / 2 + height_guard_band / 2)),
            (-(area_width / 2 + area_width_guard_band / 2), -(area_height / 2 + height_guard_band / 2)),
            (-(area_width / 2 + area_width_guard_band / 2), area_height / 2 + height_guard_band / 2),
            (area_width / 2 + area_width_guard_band / 2, area_height / 2 + height_guard_band / 2),
        ]

    corners = search_area

    # Calculate the remaining batteries to do mission
    starting_point = ((runway[0][0] + runway[3][0]) / 2, (runway[0][1] + runway[3][1]) / 2)
    estimated_distance = max([distance(x, starting_point) for x in corners])
    fail_safe_reserved_battery = battery_usage_per_meter * estimated_distance * repeat * guard_coefficient
    uav_batteries = [int(x - fail_safe_reserved_battery) for x in uav_batteries]

    # Always divide the search area with descending order
    # Closest area to the takeoff point will have smallest area and will be deployed with uav which has smallest battery
    # Furthest area to the takeoff point will have biggest area and will be deployed with uav which has biggest battery
    middle_top = ((corners[1][0] + corners[2][0]) / 2, (corners[1][1] + corners[2][1]) / 2)
    middle_bottom = ((corners[0][0] + corners[3][0]) / 2, (corners[0][1] + corners[3][1]) / 2)
    takeoff_point = ((runway[1][0] + runway[2][0]) / 2, (runway[1][1] + runway[2][1]) / 2)
    condition1 = distance(middle_top, takeoff_point) > distance(middle_bottom, takeoff_point)
    if area_width >= area_height:
        condition2 = distance((middle_bottom[1], 0), (middle_top[1], 0)) > distance((takeoff_point[1], 0),
                                                                              (middle_bottom[1], 0))
        condition3 = distance((middle_bottom[1], 0), (middle_top[1], 0)) > distance((takeoff_point[1], 0),
                                                                                    (middle_top[1], 0))
    else:
        condition2 = distance((middle_bottom[0], 0), (middle_top[0], 0)) > distance((takeoff_point[0], 0),
                                                                                    (middle_bottom[0], 0))
        condition3 = distance((middle_bottom[0], 0), (middle_top[0], 0)) > distance((takeoff_point[0], 0),
                                                                                    (middle_top[0], 0))

    temp = list(uav_batteries)
    temp.sort(reverse=True)
    if condition2 and condition3:
        temp = temp[len(temp) % 2::2] + temp[::-2]
    else:
        temp.sort(reverse=condition1)
    uav_batteries = tuple(temp)

    if area_width >= area_height:
        dist = corners[1][1] - corners[0][1] - area_bottom_guard_band - area_top_guard_band
        left_dots = []
        for i in range(0, numbers_uav + 1):
            left_dots.append(
                (corners[1][0] + area_left_guard_band, int(corners[1][1] - area_top_guard_band - dist * float(sum(uav_batteries[:i])) / float(sum(uav_batteries)))))
    else:
        dist = corners[0][0] - corners[1][0] - area_left_guard_band - area_right_guard_band
        left_dots = []
        for i in range(0, numbers_uav + 1):
            left_dots.append(
                (int(corners[1][0] + area_top_guard_band + dist * float(sum(uav_batteries[:i])) / float(sum(uav_batteries))), corners[0][1] + area_left_guard_band))

    if area_width >= area_height:
        dist = corners[2][1] - corners[3][1] - area_bottom_guard_band - area_top_guard_band
        right_dots = []
        for i in range(0, numbers_uav + 1):
            right_dots.append(
                (corners[2][0] - area_right_guard_band, int(corners[2][1] - area_top_guard_band - dist * float(sum(uav_batteries[:i])) / float(sum(uav_batteries)))))
    else:
        dist = corners[2][0] - corners[3][0] + area_left_guard_band + area_right_guard_band
        right_dots = []
        for i in range(0, numbers_uav + 1):
            right_dots.append(
                (int(corners[2][0] + area_top_guard_band - dist * float(sum(uav_batteries[:i])) / float(sum(uav_batteries))), corners[2][1] - area_top_guard_band))

    uav_area_corners = []
    if area_width >= area_height:
        for i in range(0, numbers_uav):
            uav_area_corners.append((left_dots[i + 1], left_dots[i], right_dots[i], right_dots[i + 1]))
    else:
        for i in range(0, numbers_uav):
            uav_area_corners.append((left_dots[i + 1], left_dots[i], right_dots[i], right_dots[i + 1]))

    way_points = []

    if area_width >= area_height:
        for i in range(0, numbers_uav):
            way_points.append([])
            upper_left = uav_area_corners[i][1]
            upper_right = uav_area_corners[i][2]
            bottom_left = uav_area_corners[i][0]
            bottom_right = uav_area_corners[i][3]

            k = upper_left[1] - top_guard_band
            while k >= bottom_left[1] + bottom_guard_band:
                j = upper_left[0] + left_guard_band
                counter = 0
                while j <= upper_right[0] - right_guard_band:
                    way_points[i].append((int(j), int(k)))
                    j = j + row_padding
                    counter = counter + 1
                k = k - column_padding

            # at which element will be reversed
            reverse_number = counter
            for l in range(0, len(way_points[i]), reverse_number):
                if (l / reverse_number) % 2:
                    way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))
    else:
        for i in range(0, numbers_uav):
            way_points.append([])
            upper_left = uav_area_corners[i][1]
            upper_right = uav_area_corners[i][2]
            bottom_left = uav_area_corners[i][0]
            bottom_right = uav_area_corners[i][3]

            k = upper_left[0] + top_guard_band
            while k <= bottom_left[0] - bottom_guard_band:
                j = upper_left[1] + left_guard_band
                counter = 0
                while j <= upper_right[1] - right_guard_band:
                    way_points[i].append((int(k), int(j)))
                    j = j + row_padding
                    counter = counter + 1
                k = k + column_padding

            # at which element will be reversed
            reverse_number = counter
            for l in range(0, len(way_points[i]), reverse_number):
                if (l / reverse_number) % 2:
                    way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))

    # Make sure that first way points of the each uav are as close as possible to the takeoff point
    # takeoff_point = ((runway[1][0] + runway[2][0]) / 2, (runway[1][1] + runway[2][1]) / 2)
    for i in range(0, numbers_uav):
            upper_left = uav_area_corners[i][1]
            upper_right = uav_area_corners[i][2]
            bottom_left = uav_area_corners[i][0]
            bottom_right = uav_area_corners[i][3]

            min_dist = min([distance(x, takeoff_point) for x in uav_area_corners[i]])

            if min_dist == distance(upper_left, takeoff_point):
                pass

            elif min_dist == distance(upper_right, takeoff_point):
                for l in range(0, len(way_points[i]), reverse_number):
                    way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))

            elif min_dist == distance(bottom_left, takeoff_point):
                if (len(way_points[i]) / reverse_number) % 2:
                    for l in range(0, len(way_points[i]), reverse_number):
                        way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))
                    way_points[i].reverse()
                else:
                    way_points[i] = [way_points[i][j:j + reverse_number] for j in range(0, len(way_points[i]), reverse_number)]
                    way_points[i] = way_points[i][::-1]
                    way_points[i] = sum(way_points[i], [])
                    for l in range(0, len(way_points[i]), reverse_number):
                        way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))

            elif min_dist == distance(bottom_right, takeoff_point):
                if (len(way_points[i]) / reverse_number) % 2:
                    way_points[i].reverse()
                else:
                    for l in range(0, len(way_points[i]), reverse_number):
                        way_points[i][l:l + reverse_number] = list(reversed(way_points[i][l:l + reverse_number]))
                    way_points[i].reverse()
            else:
                raise NotImplementedError

    uav_index = [x for _, x in sorted(zip(battery_capacities, range(0, 6, 1)))]
    way_points = sorted(way_points, key=len)
    temp = [None]*numbers_uav
    for i in range(0, numbers_uav):
        temp[uav_index[i]] = way_points[i]
    way_points = temp

    return way_points