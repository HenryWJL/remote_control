from kivy.support import install_twisted_reactor

install_twisted_reactor()

from twisted.internet import reactor, protocol


class EchoClient(protocol.Protocol):
    def connectionMade(self):
        self.factory.app.on_connection(self.transport)

    def dataReceived(self, data):
        pass


class EchoClientFactory(protocol.ClientFactory):
    protocol = EchoClient

    def __init__(self, app):
        self.app = app

    def startedConnecting(self, connector):
        self.app.print_message('Started to connect.')

    def clientConnectionLost(self, connector, reason):
        self.app.print_message('Lost connection.')
        self.app.reset()

    def clientConnectionFailed(self, connector, reason):
        self.app.print_message('Connection failed.')
        self.app.reset()


from kivy.app import App
from kivy.lang import Builder
from kivy.properties import StringProperty
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.clock import Clock

Builder.load_string("""
<HomeScreen>:

    ip_input: textBox
    connectButton: connect_button

    canvas.before:

        Rectangle: 
            size: self.size
            source: './data/background.png'
    
    Label:
        font_size: 50  
        center: root.width / 2 - 50, root.height / 2
        text: 'IP: '
    
    TextInput:
        id: textBox
        font_size: 30 
        size_hint: .3, .04
        cursor_color: 0, 0, 0, 1
        center: root.width / 2 + 50, root.height / 2
        multiline: False

    Button:
        id: connect_button
        font_size: 14
        size_hint: .2, .05
        background_color: 0, 1, 0, 1
        center: root.width / 2 + 50, root.height / 2 -50
        text: 'connect'
        on_press: app.connect_to_server()
        
    Label:
        font_size: 50  
        color: 1, 0, 0, 1
        center: root.width / 2, root.height / 4
        text: root.prompt
         

<ControlScreen>:

    canvas.before:

        Rectangle: 
            size: self.size
            source: './data/background.png'

    Button:
        background_normal: './data/icons/exit_arrow.png'
        size_hint: .3, .1
        center: root.width / 10, root.height / 20
        on_press: app.exit()
        
    Button:
        background_normal: './data/icons/up_arrow.png'
        size_hint: .234, .108
        center: root.width / 2, root.height / 2 - 100
        on_press: app.go_forward()
        on_release: app.stop()
        
    Button:
        background_normal: './data/icons/down_arrow.png'
        size_hint: .234, .108
        center: root.width / 2, root.height / 2 - 250
        on_press: app.go_backward()
        on_release: app.stop()
        
    Button:
        background_normal: './data/icons/left_arrow.png'
        size_hint: .234, .108
        center: root.width / 2 - 75, root.height / 2 - 175
        on_press: app.go_left()
        on_release: app.stop()

    Button:
        background_normal: './data/icons/right_arrow.png'
        size_hint: .234, .108
        center: root.width / 2 + 75, root.height / 2 - 175
        on_press: app.go_right()
        on_release: app.stop()
        
    Button:
        background_normal: './data/icons/circle.png'
        size_hint: .234, .108
        center: root.width / 2, root.height / 2 - 175
        on_press: app.grasp()
        on_release: app.stop()
""")


class HomeScreen(Screen):
    prompt = StringProperty('')


class ControlScreen(Screen):
    pass


class RemoteControlApp(App):
    connection = None
    command = StringProperty('stop')


    def build(self):
        self.root = self.setup_gui()
        Clock.schedule_interval(self.send_message, 1.0)
        return self.root


    def setup_gui(self):
        self.homeScreen = HomeScreen(name='home')
        self.controlScreen = ControlScreen(name='control')
        sm = ScreenManager()
        sm.add_widget(self.homeScreen)
        sm.add_widget(self.controlScreen)
        # sm.current = 'control'
        return sm


    def connect_to_server(self, *args):
        IP = self.homeScreen.ip_input.text
        reactor.connectTCP(IP, 8000, EchoClientFactory(self))
        
        
    def reset(self):
        self.homeScreen.ip_input.text = ''


    def on_connection(self, connection):
        self.connection = connection
        self.root.current = 'control'
    
    
    def send_message(self, *args):
        if self.connection:
            self.connection.write(self.command.encode('utf-8'))
    
    
    def print_message(self, msg):
        self.homeScreen.prompt = msg
        

    def exit(self, *args):
        self.connection.loseConnection()
        self.root.current = 'home'
        self.reset()
        
    
    def go_forward(self, *args):
        self.command = 'forward'
            
    
    def go_backward(self, *args):
        self.command = 'backward'
            
            
    def go_left(self, *args):
        self.command = 'left'
            
            
    def go_right(self, *args):
        self.command = 'right'
            
    
    def grasp(self, *args):
        self.command = 'grasp'
        
    
    def stop(self, *args):
        self.command = 'stop'

    
if __name__ == '__main__':
    RemoteControlApp().run()
