import rospy


def example_node():
    rospy.init_node("example_node", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Hello world from example_script")
        rate.sleep()


if __name__ == "__main__":
    try:
        example_node()
    except rospy.ROSInterruptException:
        pass
