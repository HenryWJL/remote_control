import rospy
from geometry_msgs.msg import Twist
from twisted.internet import reactor, protocol


class EchoServer(protocol.Protocol):
    rospy.init_node("server", anonymous=True)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    
    def dataReceived(self, data):
        command = data.decode('utf-8')
        if command == 'forward':
            message = Twist()
            message.linear.x = 1.5
            self.publisher.publish(message)
            
        elif command == 'backward':
            message = Twist()
            message.linear.x = -1.5
            self.publisher.publish(message)
            
        elif command == 'left':
            message = Twist()
            message.angular.z = 0.5
            self.publisher.publish(message)
            
        elif command == 'right':
            message = Twist()
            message.angular.z = -0.5
            self.publisher.publish(message)       
            
        self.transport.write(data)


class EchoServerFactory(protocol.Factory):
    protocol = EchoServer


if __name__ == '__main__':
    reactor.listenTCP(8000, EchoServerFactory())
    reactor.run()
