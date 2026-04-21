#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def main():
    rospy.init_node("manual_traj_selector")
    pub = rospy.Publisher("/manual_traj_selection", Int32, queue_size=1)

    print("Manual trajectory selector started.")
    print("Type an integer candidate ID and press Enter.")
    print("Ctrl+C to quit.\n")

    while not rospy.is_shutdown():
        try:
            raw = input("candidate id> ").strip()
            if raw == "":
                continue

            msg = Int32()
            msg.data = int(raw)
            pub.publish(msg)
            rospy.loginfo("Published manual trajectory selection: %d", msg.data)

        except ValueError:
            print("Please enter a valid integer.")
        except (EOFError, KeyboardInterrupt):
            break

if __name__ == "__main__":
    main()