import rospy
from rosapi.srv import Topics

'''
Topics operation
'''

def list2topic(path):
	'''
	get topic from a list of nodes
	'''
	topic = ''
	for i in path:
		topic = topic + '/' + i
	return topic

def topic2list(topic):
	'''
	get a list of nodes from a topic
	'''

	# remove the first element, it is blank
	return topic.split('/')[1:] 


def get_vrpn_body_topiclist():
	'''
	get a list of topic from /vrpn_client_node
	'''
	topics = rospy.ServiceProxy('/rosapi/topics', Topics)

	topiclist = []
	for t in topics().topics:
		if '/vrpn_client_node' in t:
			topiclist.append(t)
	return topiclist