###############################################################################
# @file        logutils.py
# @authors     Ayberk Yaraneri
#
# @brief       SILSIM data parsing and ingestion utilities
#
#
###############################################################################

import numpy as np
import re

def ingest_log(filepath):

    data_dict = {}
    event_list = []

    filehandle = open(filepath, 'r')
    for line in filehandle:
        
        # Some regex magic extracts string within [] brackets
        line_split = re.findall(r'\[([^]]*)\]', line)

        datetime_string = line_split[0]
        thread_id = line_split[1]
        log_level = line_split[2]
        component = line_split[3]

        log_message = line_split[-1].split(',')
        log_type = log_message[0]
        data = log_message[1:]
        
        if log_level == "info":
            if log_type == "DATALOG_FORMAT":
                data_dict[component] = {}

                for data_type in data:
                    data_dict[component][data_type] = []

            elif log_type == "DATA":
                for idx, data_type in enumerate(data_dict[component].keys()):
                    data_dict[component][data_type].append(float(data[idx]))

            elif log_type == "EVENT":
                event_list.append({"timestamp" : float(data[0]),
                                   "message" : data[1]})

        elif log_level == "debug":
            pass

        
    for component in data_dict.keys():
        for data_type in data_dict[component].keys():
            data_dict[component][data_type] = np.array(data_dict[component][data_type])
    
    return data_dict, event_list


if __name__ == "__main__":

    data, events = ingest_log("../logs/hmmm.log")



