#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import tkinter as tk
from tkinter import messagebox
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
import os

class PoseLabeler:
    def __init__(self):
        rospy.init_node('pose_labeler_node')
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.root = tk.Tk()
        self.root.title("Pose Labeler")

        # UI
        self.pose_label = tk.Label(self.root, text="Current Pose: (x=?, y=?, yaw=?)")
        self.pose_label.pack(pady=10)

        self.label_entry = tk.Entry(self.root, width=30)
        self.label_entry.pack()
        self.label_entry.insert(0, "Enter label here")

        self.save_button = tk.Button(self.root, text="Save Pose", command=self.save_pose)
        self.save_button.pack(pady=5)

        self.update_pose_display()
        self.root.after(1000, self.loop_tk)
        self.root.mainloop()

    def update_pose_display(self):
        try:
            trans = self.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(2.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            rot = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            self.current_pose = (x, y, yaw)
            pose_str = f"Current Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad"
            self.pose_label.config(text=pose_str)
        except Exception as e:
            self.pose_label.config(text="Pose not available")
            self.current_pose = None

        self.root.after(1000, self.update_pose_display)

    def save_pose(self):
        if not self.current_pose:
            messagebox.showerror("Error", "No pose available to save.")
            return
        
        label = self.label_entry.get().strip()
        if not label:
            messagebox.showerror("Error", "Please enter a label.")
            return

        x, y, yaw = self.current_pose
        save_str = f"{label},{x:.3f},{y:.3f},{yaw:.3f}\n"

        with open("labeled_poses.txt", "a") as f:
            f.write(save_str)

        messagebox.showinfo("Saved", f"Labeled pose saved:\n{save_str.strip()}")
        self.label_entry.delete(0, tk.END)

    def loop_tk(self):
        if not rospy.is_shutdown():
            self.root.update_idletasks()
            self.root.update()
            self.root.after(100, self.loop_tk)

if __name__ == '__main__':
    try:
        PoseLabeler()
    except rospy.ROSInterruptException:
        pass
