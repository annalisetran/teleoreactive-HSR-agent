#include "QuProlog.h"
#include "clean_the_table_foreign.hpp"

// translating from c to q prolog terms - C++ level interface between QP and C++ code
// Pass in terms that are percept terms and update arguments

// Define static percept terms
Object* robot_state_terms;

Structure* robot_base_x_term;
Structure* robot_base_y_term;
Structure* robot_base_z_term;
Structure* robot_base_r_term;
Structure* robot_base_p_term;
Structure* robot_base_yw_term;
Structure* driveable_state_term;
Structure* holding_object_id_term;
Structure* region_id_term;

Object* nil_atom; // For dynamic list allocation. 

// Used when node is closed
Object* previous_percepts;
Object* dynamic_percept;

extern "C" void *start_nodes(void);

// Action functions
extern "C" void *goto_action(char *region_name);
extern "C" void *goto_pose_action(char *frame_id, double x, double y, double z, double ox, double oy, double oz, double ow);
extern "C" void *pickup(char* class_name, int object_id);
extern "C" void *place(char *direction, char *frame_id, double x, double y, double z);

// Percept functions
extern "C" bool get_percepts(double base_position[3], double base_orientation[3],
    bool *driveable_state, int *holding_object_id, int *region_id,
    int *object_ids, int *class_ids, 
    char (*class_names)[64],
    double (*positions)[3], 
    int max_objects, int *object_count);
extern "C" bool get_robot_state_percepts(double base_position[3], 
    double base_orientation[3], 
    bool *driveable_state, int *holding_object_id, int *region_id);
extern "C" bool get_object_count(int *count);
extern "C" bool get_object_by_index(int index, int *object_id, int *class_id, double position[3], char class_name[64]);
extern "C" bool get_objects_by_classname(const char *target_class_name,
    int *object_ids, int *class_ids,
    double (*positions)[3], char (*class_names)[64],
    int max_objects, int *found_count
);

// Interface between QP level code and the C-level code for creating the nodes. Constructs
// strucutures for static terms (e.g. templates)
// Extracts terms from templates and passes them to C-level code
extern "C" bool start_nodes_interface(ForeignInterface* fi)
{
    Object* tmp;
    Object* robot_state_interface;

    // Get the fixed template terms from the X registers
    robot_state_interface = fi->getXReg(0);

    dynamic_percept = fi->getXReg(1);

    // Store references
    robot_state_terms = robot_state_interface;
    nil_atom = fi->makeAtom("[]");
    
    previous_percepts = fi->makeCons(robot_state_terms, nil_atom);

    // Extract terms from robot_state_interface 
    // traverse over cons list and only need to construct robot_state_terms once
    // Error checking occurs as traversal happens.
    if(!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_x_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_y_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_z_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_r_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_p_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    robot_base_yw_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    driveable_state_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    holding_object_id_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isCons())
        return false;
    tmp = reinterpret_cast<Cons*>(robot_state_interface)->getHead();
    if (!tmp->isStructure())
        return false;
    region_id_term = reinterpret_cast<Structure*>(tmp);
    robot_state_interface = reinterpret_cast<Cons*>(robot_state_interface)->getTail();
    if (!robot_state_interface->isNil())
        return false;   

    // Start up the ROS node
    start_nodes();
    return true;

}

// Sets the templates for percepts and also creates terms for the scene_objects
extern "C" bool get_percepts_interface(ForeignInterface* fi)
{
    Object* percept_object0;
    percept_object0 = fi->getXReg(0);

    double base_position[3];
    double base_orientation[3];
    bool driveable_state;
    int holding_object_id;
    int region_id;
    int object_ids[MAX_SCENE_OBJECTS];  
    int class_ids[MAX_SCENE_OBJECTS];
    char class_names[MAX_SCENE_OBJECTS][MAX_CLASS_NAME_LENGTH];
    double positions[MAX_SCENE_OBJECTS][3];
    int object_count;

    bool call_ok = get_percepts(base_position, base_orientation,
        &driveable_state, &holding_object_id, &region_id,
        object_ids, class_ids, class_names,
        positions, MAX_SCENE_OBJECTS, &object_count);
    
    
    if (call_ok) {
        // Update robot_state_terms
        robot_base_x_term->setArgument(1, fi->makeDouble(base_position[0]));
        robot_base_y_term->setArgument(1, fi->makeDouble(base_position[1]));
        robot_base_z_term->setArgument(1, fi->makeDouble(base_position[2]));
        robot_base_r_term->setArgument(1, fi->makeDouble(base_orientation[0]));
        robot_base_p_term->setArgument(1, fi->makeDouble(base_orientation[1]));
        robot_base_yw_term->setArgument(1, fi->makeDouble(base_orientation[2]));
        driveable_state_term->setArgument(1, fi->makeAtom(driveable_state ? "true" : "false"));
        holding_object_id_term->setArgument(1, fi->makeInteger(holding_object_id));
        region_id_term->setArgument(1, fi->makeInteger(region_id));

        // Create and update scene_objects_term
        Object* scene_objects_list = nil_atom; // Start with empty list
        for (int i = object_count - 1; i >= 0; --i) {
            Structure* position = fi->makeStructure(3);
            position->setFunctor(fi->makeAtom("position"));
            position->setArgument(1, fi->makeDouble(positions[i][0]));
            position->setArgument(2, fi->makeDouble(positions[i][1]));
            position->setArgument(3, fi->makeDouble(positions[i][2]));
            
            Structure* ob = fi->makeStructure(4);
            ob->setFunctor(fi->makeAtom("object"));
            ob->setArgument(1, fi->makeInteger(object_ids[i]));
            ob->setArgument(2, fi->makeInteger(class_ids[i]));
            ob->setArgument(3, position);
            ob->setArgument(4, fi->makeAtom(class_names[i]));
            
            scene_objects_list = fi->makeCons(ob, scene_objects_list);
        }
        Structure* scene_objects_term = fi->makeStructure(1);
        scene_objects_term->setFunctor(fi->makeAtom("scene_objects"));
        scene_objects_term->setArgument(1, scene_objects_list);

        // Assemble complete percepts list
        Object* pterm;

        pterm = robot_state_terms;
        pterm = fi->makeCons(scene_objects_term, pterm);

        previous_percepts = pterm;

        return fi->unify(percept_object0, pterm);
    }
    // call failed (because node exited) - return a default
    return fi->unify(percept_object0, previous_percepts);
}

// Interface function for goto action
// Receives: goto_region_action(RegionName) or goto_pose_action(Frame, X, Y, Z, OX, OY, OZ, OW)
extern "C" bool goto_region_interface(ForeignInterface *fi)
{
    Object* arg0 = fi->getXReg(0);
    
    char* region_name = nullptr;

    printf("Executing go to region action");

    if (arg0->isString()) {
        region_name = fi->getString(arg0);
    } else if (arg0->isAtom()) {
        region_name = fi->getAtomString(arg0);  // Convert atom to string
    } else {
        printf("goto_region: expected string or atom for region name\n");
        return false;
    }

    printf("goto_region: interface action called to %s", region_name);
    
    goto_action(region_name);
    return true;
}

extern "C" bool goto_pose_interface(ForeignInterface *fi)
{
    // Extract all 8 arguments
    Object* arg0 = fi->getXReg(0);  // frame_id
    Object* arg1 = fi->getXReg(1);  // x
    Object* arg2 = fi->getXReg(2);  // y
    Object* arg3 = fi->getXReg(3);  // z
    Object* arg4 = fi->getXReg(4);  // ox
    Object* arg5 = fi->getXReg(5);  // oy
    Object* arg6 = fi->getXReg(6);  // oz
    Object* arg7 = fi->getXReg(7);  // ow
    
    if (!arg0->isString()) {
        printf("goto_pose: expected string for frame_id\n");
        return false;
    }
    
    char* frame_id = fi->getString(arg0);
    
    auto getNumber = [](Object* obj) -> double {
        if (obj->isDouble())
            return obj->getDouble();
        else if (obj->isShort() || obj->isLong())
            return static_cast<double>(obj->getInteger());
        return 0.0;
    };
    
    double x = getNumber(arg1);
    double y = getNumber(arg2);
    double z = getNumber(arg3);
    double ox = getNumber(arg4);
    double oy = getNumber(arg5);
    double oz = getNumber(arg6);
    double ow = getNumber(arg7);

    goto_pose_action(frame_id, x, y, z, ox, oy, oz, ow);

    printf("goto_pose: frame=%s, pos=(%.2f,%.2f,%.2f)\n", frame_id, x, y, z);
    return true;
}

// Interface function for pickup action
extern "C" bool pickup_interface(ForeignInterface *fi)
{
    Object* arg0 = fi->getXReg(0);  // class_name
    Object* arg1 = fi->getXReg(1);  // object_id

    if (arg0->isAtom() == false) {
        printf("pickup: expected atom for class_name\n");
        return false;
    }
    if (arg1->isShort() == false && arg1->isLong() == false) {
        printf("pickup: expected integer for object_id\n");
        return false;
    }

    char* class_name = fi->getAtomString(arg0);
    int object_id = arg1->getInteger();

    pickup(class_name, object_id);

    return true;
}

// Interface function for place action
extern "C" bool place_interface(ForeignInterface *fi)
{
    // Extract arguments: place_action(Direction, FrameId, X, Y, Z)
    Object* arg0 = fi->getXReg(0);  // direction
    Object* arg1 = fi->getXReg(1);  // frame_id (now string)
    Object* arg2 = fi->getXReg(2);  // x
    Object* arg3 = fi->getXReg(3);  // y
    Object* arg4 = fi->getXReg(4);  // z
    
    char* direction = nullptr;
    if (arg0->isString()) {
        direction = fi->getString(arg0);
    } else if (arg0->isAtom()) {
        direction = fi->getAtomString(arg0);
    } else {
        printf("place: expected string or atom for direction\n");
        return false;
    }

    if (!arg1->isString()) {
        printf("place: expected string for frame_id\n");
        return false;
    }

    char* frame_id = fi->getString(arg1);

    auto getNumber = [](Object* obj) -> double {
        if (obj->isDouble()) return obj->getDouble();
        else if (obj->isShort() || obj->isLong()) 
            return static_cast<double>(obj->getInteger());
        return 0.0;
    };
    
    double x = getNumber(arg2);
    double y = getNumber(arg3);
    double z = getNumber(arg4);
    
    place(direction, frame_id, x, y, z);
    printf("place: direction=%s, frame=%s, pos=(%.2f,%.2f,%.2f)\n", 
        direction, frame_id, x, y, z);
    return true;
}