"""
"""

import xmltodict
from pathlib import Path
import pandas as pd
import json

out_dict = {}

def get_length(xml_filename):
    in_file = open(xml_filename, 'r')
    xml_string = in_file.read()
    in_file.close()
    xml_dict = xmltodict.parse(xml_string)

    cur_len = 0

    all_subcomponents = xml_dict["openrocket"]["rocket"]["subcomponents"]["stage"]["subcomponents"]\
    
    for key in all_subcomponents:
        cur_val = all_subcomponents[key]

        if(type(cur_val) is dict):
            cur_len += float(cur_val["length"])
        else:
            for component in cur_val:
                cur_len += float(component["length"])
    
    out_dict["length"] = cur_len


def get_other_values(csv_file_name):
    csv_in = pd.read_csv(csv_file_name, comment="#",on_bad_lines='skip')
    dry_cg = 0

    for i in range(len(csv_in["CG location (cm)"])):
        if(i<len(csv_in["CG location (cm)"])-1):
            if(pd.isnull(csv_in["CG location (cm)"].iloc[i+1]) and dry_cg==0):
                dry_cg = csv_in["CG location (cm)"].iloc[i]
    
    out_dict["dry_center_of_gravity"] = dry_cg/100

    out_dict["wet_center_of_gravity"] = csv_in["CG location (cm)"].iloc[0]/100

    out_dict["diameter"] = csv_in["Reference length (cm)"].iloc[0]/100

    out_dict["wet_mass"] = csv_in["Mass (g)"].iloc[0]/1000

    out_dict["dry_mass"] = (csv_in["Mass (g)"].iloc[0]-csv_in["Motor mass (g)"].iloc[0])/1000

if __name__ == "__main__":
    p = Path(__file__).parents[1]
    
    with open(str(p)+"/ork_files/rocket_csv.csv") as csv_file:
        data = csv_file.readlines()

    for i in range(len(data)):
        if "Mass" in data[i] and data[i][0]=="#": 
            data[i] = data[i][1:]
    
    with open(str(p)+"/ork_files/rocket_csv.csv", "w") as csv_file:
        csv_file.writelines(data)

            
    get_other_values(str(p)+"/ork_files/rocket_csv.csv")
    get_length( str(p)+"/ork_files/rocket.ork")

    output_string = ""
    for k,v in out_dict.items():
        output_string += str(k) + " " + str(v) + "\n"
    
    with open('src/Rocket/values.txt', 'w') as f:
        f.writelines(output_string)