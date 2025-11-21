## World Model APIs

### Regions
#### File: regions_api.py
The regions API contains methods for manipulating region data

#### Methods

The common usage code is required at the beginning of the python file that will use the region_api
Common Usage:
```
from db import Db
from regions_api import RegionsApi

db = Db(os.path.abspath("/home/adam/.ros/world_model_kb/database.ini"))               # database connections

regions = RegionsApi()

```

##### get_poi_list
This method returns the list of all points of interest.

Parameters:     none

Return:         Python List of all points of interest (point_of_interest_id, name, loc_x, loc_y, loc_z, region) in the database as a json format string

Usage:
```
test = regions.get_poi_list(db.con_pool)
```

