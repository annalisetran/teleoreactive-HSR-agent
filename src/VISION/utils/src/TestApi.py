import os
import sys
import cv2
from cv_bridge import CvBridge
from ObjectFeatureExtractor import ObjectFeatureExtractor
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")
from dbase import DBase
from object_class_api import ObjectClassApi

db_ini = "$(env UNSW_WS)/WORLD_MODEL/world_model/world_model_kb/scripts/database.ini"
ofe = ObjectFeatureExtractor(debug=True, db_ini=db_ini, shape_w=128, shape_h=128, feat_threshold=0.65)
obj_class = ObjectClassApi()
db = DBase(os.path.abspath(db_ini))               # database connections
image_dir = ""

## add test scripts here

# loop through files in each folder
for dirpath, dirnames, filenames in os.walk(image_dir):
    print(f"Found directory: {dirpath} with {len(filenames)} images")
    for filename in filenames:
        if filename.endswith(".jpg") or filename.endswith(".png"):
            img_path = os.path.join(dirpath, filename)
            img = cv2.imread(img_path)

            # add feature extractor
            feat = ofe.extract_feature()
            ofe.insert_feature(feat, )


