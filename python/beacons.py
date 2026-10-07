import math

from cpp_random import mt19937, uniform_real_distribution
from display import Vector2, offsetPoints


class BeaconData:
    def __init__(self, xPos=0.0, yPos=0.0, beaconId=-1):
        self.x = xPos
        self.y = yPos
        self.id = beaconId


class BeaconMap:

    def __init__(self):
        self.m_beacon_map = []
        rand_gen = mt19937()
        pos_dis = uniform_real_distribution(-500.0, 500.0)
        for i in range(200):
            # g++ evaluates function arguments right-to-left, so in the C++ code
            # addBeacon(pos_dis(rand_gen),pos_dis(rand_gen)) the y value is drawn first.
            y = pos_dis(rand_gen)
            x = pos_dis(rand_gen)
            self.addBeacon(x, y)

    def addBeacon(self, x, y):
        self.m_beacon_map.append(BeaconData(x, y, len(self.m_beacon_map)))

    def getBeaconWithId(self, id):
        for beacon in self.m_beacon_map:
            if beacon.id == id:
                return BeaconData(beacon.x, beacon.y, beacon.id)
        return BeaconData()

    def getBeaconsWithinRange(self, x, y, range):
        beacons = []
        for beacon in self.m_beacon_map:
            delta_x = beacon.x - x
            delta_y = beacon.y - y
            beacon_range = math.sqrt(delta_x * delta_x + delta_y * delta_y)
            if beacon_range < range:
                beacons.append(BeaconData(beacon.x, beacon.y, beacon.id))
        return beacons

    def getBeacons(self):
        return [BeaconData(b.x, b.y, b.id) for b in self.m_beacon_map]

    def render(self, disp):
        beacon_lines = [Vector2(1, 0), Vector2(0, 1), Vector2(0, -1), Vector2(1, 0)]
        disp.setDrawColour(255, 255, 0)
        for beacon in self.m_beacon_map:
            disp.drawLines(offsetPoints(beacon_lines, Vector2(beacon.x, beacon.y)))
