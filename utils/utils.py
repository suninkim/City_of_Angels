import math

PI = math.pi

def rad2deg(rad):
    return rad * 180.0 / PI

def deg2rad(deg):
    return deg * PI / 180.0


def convert_os_command(command):
    return command