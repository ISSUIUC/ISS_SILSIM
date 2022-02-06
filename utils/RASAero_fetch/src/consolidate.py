##
#  @file        consolidate.py
#  @authors     Kenneth Tochihara
# 
#  @brief       Consolidation of RASAero data collected
#
 
from os import listdir
import numpy as np
from operator import itemgetter
import csv

output_directory = "output"

# indicates (header_name, row_idx, col_idx) for each grouping
headers = [("Mach Number", 0, 0), 
           ("Alpha (deg)", 4, 1), 
           ("Protuberance (%)", -1, -1),
           ("CD Power-Off", 4, 3), 
           ("CD Power-On", 5, 3), 
           ("CA Power-Off", 4, 5), 
           ("CA Power-On", 5, 5), 
           ("CN Total", 3, 2), 
           ("CP Total", 3, 3)]


## 
#  @brief Parses from `output` directory with `XX_XXX.txt` file name
# 
#  @param directory Directory path of RASAero-generated files
#  @param filename Filename of RASAero-generated files
# 
#  @return List of dictionaries of data gathered from file
def parse_file(directory, filename):
    
    # open file
    with open(directory + "/" + filename, 'r') as f:
        data = f.readlines()
    
    # split by groupings
    data = "\n".join(data)
    data = data.split("\n\n\n\n")
    
    # ignore transitional headers and empty entries
    data = [line for line in data if not '-------->' in line]
    data = [line for line in data if line.strip()]
    
    # split groupings into specific numerical entries
    refined_data = []
    for line in data:  
        line = list(filter(None, line.split("\n")))
        line = [np.array(entry.split()).astype(np.float64) for entry in line]
        refined_data.append(line)
    data = refined_data
    
    # build list of dictionaries
    parsed_output = []
    for entry in data:
        
        # build dictionary entry
        entry_dictionary = {}
        for header in headers:
            
            if header[0] == "Protuberance (%)":
                entry_dictionary[header[0]] = get_protuberance(filename)
            else:
                entry_dictionary[header[0]] = entry[header[1]][header[2]]
            
        parsed_output.append(entry_dictionary)
    return parsed_output

## 
#  @brief Parses protuberance percentage from filename
# 
#  @param filename Filename of RASAero-generated file
# 
#  @return Integer percentage of protuberance
def get_protuberance(filename):
    return int(filename.split("_")[1].strip(".txt"))/100

## 
#  @brief Parses through all files generated by RASAero in `output` directory
# 
#  @return Sorted list of dictionaries for all files
def generate_consolidated_data():
    
    # loop thorugh each file and accumulate entries
    consolidated_data = []  
    for filename in sorted(listdir(output_directory)):
        if "RASAero.csv" in filename: continue
        consolidated_data += parse_file(output_directory, filename)
        
    # return sorted dictionary entries by mach, aoa, and protuberancez
    return sorted(consolidated_data, key=itemgetter('Mach Number', "Alpha (deg)", "Protuberance (%)"))

## 
#  @brief Saves parsed data as consolidated CSV file
# 
#  @param consolidated_data List of dictionaries containing all RASAero generated data
#  @param directory Consolidated data CSV save directory path
#  @param filename Consolidated data CSV save filename
def save_conlidated_data(consolidated_data, directory, filename):
    
    # save as csv
    with open(directory + "/" + filename, "w") as f:
        dict_writer = csv.DictWriter(f, consolidated_data[0].keys())
        dict_writer.writeheader()
        dict_writer.writerows(consolidated_data)
    
def main():
    consolidated_data = generate_consolidated_data()
    save_conlidated_data(consolidated_data, output_directory, "RASAero.csv")
    
if __name__ == "__main__":
   main()