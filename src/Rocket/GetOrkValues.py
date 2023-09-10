"""
Rocket Object

An object to store all components of a rocket. Also contains all XML parsing
and CSV output functionality.
"""

from components import *
import xmltodict
from pathlib import Path
import json

class Rocket:
    def __init__(self, name, xml_filename):
        self.name = name
        self.xml_filename = xml_filename
        in_file = open(xml_filename, 'r')
        xml_string = in_file.read()
        in_file.close()
        xml_dict = xmltodict.parse(xml_string)

        # for i, sub_component in xml_dict["openrocket"]["rocket"]["subcomponents"]['stage']['subcomponents']:
        #     print(name, type(sub_component))

        # print(xml_dict["openrocket"]["rocket"]["subcomponents"]['stage']['subcomponents'])
        sub_c = xml_dict["openrocket"]["rocket"]["subcomponents"]['stage']['subcomponents']

        json_object = json.dumps(sub_c, indent=4)

        with open("sample.json", "w") as outfile:
            outfile.write(json_object)
        
        print(type(sub_c["bodytube"]))
        for t in sub_c["bodytube"]:
            print(t["name"])
        
        # print(sub_c['nosecone'].keys())

        # for c in sub_c:
        #     if 'subcomponents' in sub_c[c]:
        #         print(sub_c[c], 'has subcomponents')
        #     # print(sub_c[c])
        loop_through(sub_c)

def loop_through(dict_in):
    for a in dict_in:
        if type(dict_in[a]) is dict:
            loop_through(dict_in[a])
        else:
            # print(a)
            pass

if __name__ == "__main__":
    p = Path(__file__).parents[2]
    # print(p)

    Rocket("bruh", str(p)+"/ork_files/rocket-1.ork")