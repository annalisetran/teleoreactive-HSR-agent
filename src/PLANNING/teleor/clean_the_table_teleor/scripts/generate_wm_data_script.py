#!/usr/bin/env python3
"""
Continuously publish test data to the world_model database
for testing the TeleoR connection.

Usage: python3 publish_test_data.py
"""

import psycopg2
import time
import math
from datetime import datetime

# Database connection parameters
DB_PARAMS = {
    'dbname': 'world_model',
    'user': 'postgres',
    'password': '',  # Add password if needed
    'host': 'localhost',
    'port': 5432
}

def connect_db():
    """Connect to the PostgreSQL database."""
    try:
        conn = psycopg2.connect(**DB_PARAMS)
        print("✓ Connected to database")
        return conn
    except Exception as e:
        print(f"✗ Error connecting to database: {e}")
        return None

def initialize_robot_state(conn):
    """Initialize the robot_state table with a current entry."""
    try:
        cur = conn.cursor()
        
        # Set all existing entries to not current
        cur.execute("UPDATE robot_state SET is_current = 0")
        
        # Insert initial robot state
        cur.execute("""
            INSERT INTO robot_state 
                (frame_id, base_pose_x, base_pose_y, base_pose_z, 
                 base_r, base_p, base_yw,
                 driveable_state, holding_object_id, region_id, is_current, created)
            VALUES 
                ('map', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true, NULL, 1, 1, NOW())
        """)
        
        conn.commit()
        print("✓ Initialized robot_state table")
        cur.close()
    except Exception as e:
        print(f"✗ Error initializing robot state: {e}")
        conn.rollback()

def initialize_objects(conn):
    """Initialize some test objects in the database."""
    try:
        cur = conn.cursor()
        
        # Check if object_class table exists and has data
        cur.execute("SELECT COUNT(*) FROM object_class")
        if cur.fetchone()[0] == 0:
            # Insert some test object classes
            cur.execute("""
                INSERT INTO object_class (object_class_id, object_class_name) 
                VALUES 
                    (1, 'plate'),
                    (2, 'fork'),
                    (3, 'cup'),
                    (4, 'bowl')
                ON CONFLICT DO NOTHING
            """)
            print("✓ Added test object classes")
        
        conn.commit()
        cur.close()
    except Exception as e:
        print(f"✗ Error initializing objects: {e}")
        conn.rollback()

def update_robot_state(conn, t):
    """Update robot state with simulated movement."""
    try:
        cur = conn.cursor()
        
        # Simulate robot moving in a circle
        x = 2.0 * math.cos(t / 10.0)
        y = 2.0 * math.sin(t / 10.0)
        z = 0.0
        yaw = t / 10.0
        
        # Simulate changing regions (kitchen=1, dining=2)
        region = 1 if math.sin(t / 20.0) > 0 else 2
        
        # Don't simulate holding for now - just use NULL
        holding = None
        
        # Set all to not current
        cur.execute("UPDATE robot_state SET is_current = 0")
        
        # Insert new current state
        cur.execute("""
            INSERT INTO robot_state 
                (frame_id, base_pose_x, base_pose_y, base_pose_z, 
                 base_r, base_p, base_yw,
                 driveable_state, holding_object_id, region_id, is_current, created)
            VALUES 
                ('map', %s, %s, %s, 0.0, 0.0, %s, true, %s, %s, 1, NOW())
        """, (x, y, z, yaw, holding, region))
        
        conn.commit()
        cur.close()
        return x, y, z, region, holding
        
    except Exception as e:
        print(f"✗ Error updating robot state: {e}")
        conn.rollback()
        return None, None, None, None, None

def update_scene_objects(conn, t):
    """Update scene objects with simulated movement."""
    try:
        cur = conn.cursor()
        
        # Set all current objects to not current
        cur.execute("UPDATE object SET is_current = 0")
        
        # Create some test objects that move around
        objects = [
            {'class_id': 1, 'name': 'plate', 'x': 1.5 + 0.1 * math.sin(t/5), 'y': 2.0, 'z': 0.8},
            {'class_id': 2, 'name': 'fork', 'x': 1.7, 'y': 2.1 + 0.1 * math.cos(t/5), 'z': 0.8},
            {'class_id': 3, 'name': 'cup', 'x': 1.3, 'y': 1.9, 'z': 0.85},
            {'class_id': 4, 'name': 'bowl', 'x': 1.6, 'y': 2.2, 'z': 0.8},
        ]
        
        # Insert or update each object
        for obj in objects:
            cur.execute("""
                INSERT INTO object 
                    (object_class_id, class_confidence, frame_id, 
                     loc_x, loc_y, loc_z, 
                     created, last_updated, last_seen, is_current)
                VALUES 
                    (%s, 0.95, 'map', %s, %s, %s, NOW(), NOW(), NOW(), 1)
            """, (obj['class_id'], obj['x'], obj['y'], obj['z']))
        
        conn.commit()
        cur.close()
        return len(objects)
        
    except Exception as e:
        print(f"✗ Error updating scene objects: {e}")
        conn.rollback()
        return 0

def main():
    """Main loop to continuously publish test data."""
    print("=" * 60)
    print("Test Data Publisher for world_model Database")
    print("=" * 60)
    
    conn = connect_db()
    if not conn:
        return
    
    # Initialize tables
    initialize_robot_state(conn)
    initialize_objects(conn)
    
    print("\n✓ Starting continuous updates (Ctrl+C to stop)")
    print("-" * 60)
    
    t = 0
    update_count = 0
    
    try:
        while True:
            # Update robot state
            x, y, z, region, holding = update_robot_state(conn, t)
            
            # Update scene objects
            obj_count = update_scene_objects(conn, t)
            
            if x is not None:
                update_count += 1
                status = f"✓ Update #{update_count:04d}: "
                status += f"Pos=({x:5.2f}, {y:5.2f}, {z:5.2f}) "
                status += f"Region={region} "
                status += f"Objects={obj_count} "
                status += f"Holding={'Yes' if holding is not None else 'No '}"
                print(status)
            
            # Wait before next update
            time.sleep(1.0)
            t += 1
            
    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        if conn:
            conn.close()
            print("✓ Database connection closed")

if __name__ == "__main__":
    main()