import random


class Vehicle:

    all_vehicle = list()

    def __init__(self, ego_id):
        self.m_id = ego_id
        self.buffer = None
        print("Vehicel ", self.m_id, " construct.")
        Vehicle.all_vehicle.append(self)

    def __del__(self):
        print("Vehcile ", self.m_id, " destruct.")
        Vehicle.all_vehicle.remove(self)

    def get_data(self):
        self._calculate_return()
        return self.buffer



