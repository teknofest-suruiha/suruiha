import math
import tf


def clamp(val, min_val, max_val):
    if val > max_val:
        return max_val
    elif val < min_val:
        return min_val
    else:
        return val


def distance(x1, y1, z1, x2, y2, z2):
    sum = math.pow(x2-x1,2) + math.pow(y2-y1,2) + math.pow(z2-z1,2)
    return math.sqrt(sum)


def to_euler(quat):
    quaternion = (quat.x, quat.y, quat.z, quat.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler


def normalize_angle(radian_val):
    if radian_val < -math.pi:
        radian_val += 2*math.pi
    elif radian_val > math.pi:
        radian_val -= 2*math.pi
    return radian_val