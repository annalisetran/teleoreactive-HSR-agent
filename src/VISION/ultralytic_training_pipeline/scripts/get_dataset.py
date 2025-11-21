from segments import SegmentsClient, SegmentsDataset
from segments.utils import export_dataset

'''
This script demonstrates how to export a Segments dataset to YOLO compatable format.
It requires the Segments Python client library, which can be installed via pip:
pip install segments-ai

To run this script, you will need to replace the project ID and release ID with your own unless you use Adam Golding's account for labelling.
'''
# Initialize a SegmentsDataset from the release file
api_key = input("Enter your API key(press enter if you use A.Golding's Segments AI account): ")
if api_key.strip() == "":
    api_key = '0ff4c4791d8952b89538e32350b5a286c3925013'

repo_name = input("Enter your repository name: ")
release_name = input("Enter your release name: ")

dataset_name = input("Enter the name you want your dataset to be: ")

client = SegmentsClient(api_key)
release = client.get_release(repo_name, release_name)
dataset = SegmentsDataset(release, labelset='ground-truth', 
filter_by=['labeled', 'reviewed'])


# Export to COCO panoptic format
# specify the path to save the exported dataset

out_path = export_dataset(dataset, export_format='yolo')

# The exported dataset will be saved in the 'temp/segments/{reponame}/{versionname}' folder in the current directory
# move it to raw_data/{dataset_name} folder
import os
os.system(f"mkdir -p raw_data/{dataset_name}")

os.system(f"mv {out_path}/* raw_data/{dataset_name}")

#agolding/athome-robocup2024-bag