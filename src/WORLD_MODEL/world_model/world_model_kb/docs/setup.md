# World Model Store

## Setup

### PostgreSQL
PostgreSQL is the database platform used for the world model database store. 

To check if PostgreSQL is installed 
```
sudo service postgresql status
```

The following website provides the relevant details to install PostgreSQL on Ubuntu
https://www.postgresql.org/download/linux/ubuntu/

PostgreSQL Documentation can be found here:
https://www.postgresql.org/docs/

#### Starting PostgreSQL
```
sudo service postgresql start

sudo service postgresql restart
```

#### Stopping PostgreSQL
```
sudo service postgresql stop
```

### Usefull Database Tools
#### PGAdmin4
Is a GUI Database Admin tool. You can make changes to database structures, manage users, write and test queries. Be careful how you install this as there is an option to install just the desktop version or also include the web version. Install only the desktop version as the web version isn't required. 
https://www.pgadmin.org/download/pgadmin-4-apt/


### Database Authentication

This method is arguably insecure (but very easy) - feel free to do this in a proper way

In /etc/postgresql/\<version\>/main/pg_hba.conf change the following lines:
  
```  
  # Database administrative login by Unix domain socket
  #local   all             postgres                                md5
  local   all             postgres                                peer

  # More stuff in the middle
  
  # IPv4 local connections:
  #host    all             all             127.0.0.1/32            md5
  host    all             all             127.0.0.1/32            trust
```

Then reload the database service to read in the config changes
```
sudo service postgresql reload
sudo service postgresql restart
```

### Creating the World Model Database
```
sudo -u postgres psql
CREATE DATABASE world_model;
```

Provide your user with privlidges
```
sudo -u postgres createuser your_username
sudo -u postgres psql
GRANT ALL PRIVILEGES ON DATABASE world_model TO your_username;
```

Now, setup tables:
```
# Replace <version> with the most recent version
psql world_model -f database/setup_world_model_<version>.sql
psql world_model -f database/setup_insert_<version>.sql
```

### Database Connection
Default launch file expects a database.ini config file at ~/.ros/world_model_kb/database.ini

An example database.ini is shown below:

```

[postgresql]
host=localhost
database=world_model
user=wm_user
password=wm_user

[pool]
min=1
max=5

```

### Required Python Packages
db.py 
https://pypi.org/project/db.py
```
pip install db.py
```
psycopg2
https://phpi.org/project/psycopg2.py
```
pip install psycopg2
```

Filterpy
https://filterpy.readthedocs.io/en/latest
```
pip install filterpy
```

Tabulate
https://pip.org/project/tabulate
```
pip install tabulate
```

### Source Code

Clone into a catkin workspace src dir e.g. ~/workspace/noetic_workspace/src


## Usage

roslaunch world_model_kb world_model.launch

There should then be a set of services, /world_model/*

For testing, you can call these using rosservice call ...