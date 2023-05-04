from data_handling import get_data, get_data_json
import matplotlib.pyplot as plt
import json

data_directory = '../recorded_bags/paper_simulation/'
data_name = 'bad_initialization_failed_at_second_part'
extension = '.bag'
datafile = data_directory + data_name + extension

# Get data
data = get_data(datafile)
datafile_noext = datafile.replace('.bag', '')

# Store data from bag->csv file to .json
json_data = json.dumps(data)
with open(datafile_noext+'.json', "w") as outfile:
    outfile.write(json_data)

print(" ")
print("Data is now stored as\n" + datafile_noext + '.json')
