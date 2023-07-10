import rospy
from geometry_msgs.msg import Twist
from twisted.internet import reactor, protocol


class EchoServer(protocol.Protocol):
    
    
    def __init__(self, publisher, rate):
        super(EchoServer, self).__init__()
        self.publisher = publisher
        self.rate = rate
    
    
    def dataReceived(self, data):
        command = data.decode('utf-8')
        if command == 'forward':
            message = Twist()
            message.linear.x = 1.5
            self.publisher.publish(message)
            rate.sleep()
            
        elif command == 'backward':
            message = Twist()
            message.linear.x = -1.5
            self.publisher.publish(message)
            rate.sleep()
            
        elif command == 'left':
            message = Twist()
            message.angular.z = 0.5
            self.publisher.publish(message)
            rate.sleep()
            
        elif command == 'right':
            message = Twist()
            message.angular.z = -0.5
            self.publisher.publish(message)
            rate.sleep()       


class EchoServerFactory(protocol.Factory):
    def __init__(self, publisher, rate):
        super(EchoServerFactory, self).__init__()
        self.protocol = EchoServer(publisher, rate)


if __name__ == '__main__':
    rospy.init_node("server", anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(5)
    reactor.listenTCP(8000, EchoServerFactory(pub, rate))
    reactor.run()
