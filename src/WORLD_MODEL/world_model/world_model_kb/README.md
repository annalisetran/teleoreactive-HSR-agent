# World Model Store

## Description

The SOAR architecture describes the world model as the symbolic working memory where the 
unique identity of objects are created and features describing the object are associated with it.

The world model is implemented as a relational database and contains information on the following concepts:

object classes
object attributes
objects (instances of object classes)
object relationships
scenes that contain objects
regions 
points of interest
robot state
agents (people in the environment)

**NOTE:** This version of the World Model APIs utilise 2d bounding boxes as per unsw_vision_msg (BoundingBox bbox)

### Table of Contents
| Topic      | Description |
|------------|-------------|
| [Setup](docs/setup.md)      | Describes the dependencies, requirements and setup procedures for creating the world model |
| [APIs](docs/api.md)       | Descripbes how to use the APIs for the World Model |
| [DB Schema](docs/dbschema.md)  | Describes the database schema with data dictionary |
