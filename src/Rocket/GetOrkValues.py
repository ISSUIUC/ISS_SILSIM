"""
Rocket Object

An object to store all components of a rocket. Also contains all XML parsing
and CSV output functionality.
"""

from components import *
import xmltodict

if __name__ == "__main__":
    pass

class Rocket:
    def __init__(self, name, xml_filename):
        self.name = name
        self.xml_filename = xml_filename
        in_file = open(xml_filename, 'r')
        xml_string = in_file.read()
        in_file.close()
        xml_dict = xmltodict.parse(xml_string)

        self.mass = xml_dict["openrocket"]["simulations"]["simulation"][0]["flightdata"]["databranch"]["datapoint"][1]
        print(self.mass, "this is the mass")

        # Populating data

        nosecone_dict = xml_dict["openrocket"]["rocket"]["subcomponents"]["stage"]["subcomponents"]["nosecone"]
        self.nosecone = Nosecone(float(nosecone_dict["length"]),
                                       nosecone_dict["shape"],
                                       nosecone_dict["material"]["#text"])
        self.sims = []
        sim_list = []
        try:
            sim_list = xml_dict["openrocket"]["simulations"]["simulation"]
        except:
            print('no sims')

        for sim in sim_list:
            try:
                self.sims.append(Simulation(sim["flightdata"]["@maxaltitude"],
                                            sim["flightdata"]["@maxvelocity"],
                                            sim["flightdata"]["@maxacceleration"],
                                            sim["flightdata"]["@flighttime"]))
            except:
                print("unparsable sim")
                pass

        tube_collection = xml_dict["openrocket"]["rocket"]["subcomponents"]["stage"]["subcomponents"]["bodytube"]
        self.tubes = []
        if isinstance(tube_collection, list):
            for tube in tube_collection:
                self.tubes.append(BodyTube(tube['name'],float(tube['length']), tube['material']['#text'], float(tube['thickness'])))
                try:
                    fin_dict = tube['subcomponents']['trapezoidfinset']
                    self.finset = Finset('trapezoid', fin_dict["fincount"], fin_dict["material"]["#text"], float(fin_dict["height"]), float(fin_dict["rootchord"]), float(fin_dict["tipchord"]))
                except:
                    print("this tube has no trapezoid fins")

                try:
                    fin_dict = tube['subcomponents']['freeformfinset']
                    self.finset = Finset("freeform", fin_dict['fincount'],fin_dict['material']['#text'])
                except:
                    print('this tube has no freeform fins')
                # try:
                #     recovery_dict =
                #     self.RecoverySystem =
                # SOMETHING TO HANDLE DUALITY OF DICT/LIST BETTER
        else:
            self.tubes.append(BodyTube(tube_collection['name'],float(tube_collection['length']), tube_collection['material']['#text'], float(tube_collection['thickness'])))
            try:
                fin_dict = tube_collection['subcomponents']['trapezoidfinset']
                self.finset = Finset('trapezoid', fin_dict["fincount"], fin_dict["material"]["#text"], float(fin_dict["height"]), float(fin_dict["rootchord"]), float(fin_dict["tipchord"]))
            except:
                print("this tube has no trapezoid fins")

            try:
                fin_dict = tube_collection['subcomponents']['freeformfinset']
                self.finset = Finset("freeform", fin_dict['fincount'],fin_dict['material']['#text'])
            except:
                print('this tube has no freeform fins')
