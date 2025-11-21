-- Reset sequence to a specific value (e.g., 1)
-- the following lines represent the sequences currently in the database
ALTER SEQUENCE agent_agent_id_seq RESTART WITH 1;
ALTER SEQUENCE object_object_id_seq RESTART WITH 1;
ALTER SEQUENCE point_of_interest_point_of_interest_id_seq RESTART WITH 1;
ALTER SEQUENCE region_region_id_seq RESTART WITH 1;
ALTER SEQUENCE robot_state_robot_state_id_seq RESTART WITH 1;

-- the following queries delete the contents of the agent, agent_features tables
-- data agent_features needs to be deleted first as there is a foriegn key reference to the agent table
delete from agent_feature;
delete from agent;

delete from scene_object;
delete from scene;
delete from object;

