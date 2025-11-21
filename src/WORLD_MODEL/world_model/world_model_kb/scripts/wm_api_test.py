import os
import sys
from dbase import DBase
from object_class_api import ObjectClassApi
module_path = os.environ.get("UNSW_WS")
sys.path.append(module_path + "/WORLD_MODEL/world_model/world_model_kb/scripts")

obj_class = ObjectClassApi()
db = DBase(os.path.abspath("$(env UNSW_WS)/WORLD_MODEL/world_model/world_model_kb/scripts/database.ini"))               # database connections


####  add manual scripts here

