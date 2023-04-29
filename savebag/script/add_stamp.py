import rosbag
import rospy
if __name__ == '__main__':
	rospy.init_node('node', anonymous=True)
	input_file = '/home/ron/work/src/savebag/bag/sim_bag_.bag'
	output_file = '/home/ron/work/src/savebag/bag/sim_bag_1.bag'
	bag = rosbag.Bag(input_file)
	bag_w  = rosbag.Bag(output_file, 'w')
#	topic_list = [] # Empty means read all msgs
	topic_list = ['/scan', '/camera/depth/image_raw', '/camera/rgb/image_raw', '/odom', '/joint_states']

	for topic, msg, t in bag.read_messages(topics = topic_list):
		# print("Reading message \"" + topic + "\" from " + input_file)
		bag_w.write(topic, msg, msg.header.stamp)
		# print("\"" + topic + "\" is successfully written into " + output_file)

	bag.close()
	bag_w.close()