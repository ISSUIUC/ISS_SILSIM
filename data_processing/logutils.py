###############################################################################
# @file        logutils.py
# @authors     Ayberk Yaraneri
#
# @brief       SILSIM data parsing and ingestion utilities
#
#
###############################################################################

import numpy as np

def ingest_log(filepath):

    master_dict = {}

    filehandle = open(filepath, 'r')
    for line in filehandle:
        
        line_split = line.strip('\n').split(" ")
        date_string = line_split[0].strip('[').strip(']')
        time_string = line_split[1].strip('[').strip(']')
        component = line_split[2].strip('[').strip(']')
        log_level = line_split[3].strip('[').strip(']')
        line_type = line_split[4].strip('[').strip(']')
        data = line_split[-1].split(',')
        
        if log_level == "info":
            if line_type == "DATALOG_FORMAT":
                master_dict[component] = {}

                print("Adding " + component + " to master_dict components")

                for data_type in data:
                    master_dict[component][data_type] = []

            elif line_type == "DATA":
                for idx, data_type in enumerate(master_dict[component].keys()):
                    master_dict[component][data_type].append(float(data[idx]))

        elif log_level == "debug":
            pass

        
    for component in master_dict.keys():
        for data_type in master_dict[component].keys():
            master_dict[component][data_type] = np.array(master_dict[component][data_type])
    
    return master_dict


if __name__ == "__main__":

    data = ingest_log("../logs/hmmm.log")



