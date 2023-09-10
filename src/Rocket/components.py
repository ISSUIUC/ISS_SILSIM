"""
Rocket Components

A few objects that make creating and using this tool easier than reading out of
a dictionary each time information is needed. Add more components as required. 
"""

class Nosecone:
    def __init__(self, length, geometry, material):
        self.length = length
        self.geometry = geometry
        self.material = material
    def __str__(self):
        return f"Nosecone - Length:{self.length},Geometry:{self.geometry},Material:{self.material}"

class Finset:
    def __init__(self, type, fin_count, material, height=None, root_chord=None, tip_chord=None):
        self.type = type
        self.fin_count = fin_count
        self.height = height
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.material = material
    def __str__(self):
        return f"Finset - Fin Count:{self.fin_count},Type:{self.type},Material:{self.material}"

class Simulation:
    def __init__(self, apogee, max_velocity, max_acceleration, flight_time):
        self.apogee = apogee
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.flight_time = flight_time
    def __str__(self):
        return f"Simulation - Apogee:{self.apogee},Max Velocity:{self.max_velocity},Max Acceleration:{self.max_acceleration}"

class BodyTube:
    def __init__(self, name, length, material, thickness):
        self.name = name
        self.length = length
        self.material = material
        self.thickness = thickness
    def __str__(self):
        return f"{self.name} - Length:{self.length},Material:{self.material},Thickness:{self.thickness}"

class RecoverySystem:
    def __init__(self, shockcord_length, chute_material):
        self.shockcord_length = shockcord_length
        self.chute_material = chute_material
    def __str__(self):
        return f"RecoverySystem - Shockcord Length:{self.shockcord_length},Chute Material:{self.chute_material}"
