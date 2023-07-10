from __future__ import unicode_literals

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
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import StringProperty
from kivy.uix.screenmanager import Screen, ScreenManager

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
        center: root.width / 2 + 50, root.height / 2
        multiline: False

    Button:
        id: connect_button
        font_size: 14
        size_hint: .2, .05
        center: root.width / 2 + 50, root.height / 2 -50
        text: 'connect'
        
        
    Label:
        font_size: 50  
        center: root.width / 2, root.height / 4
        text: root.prompt
         

<ControlScreen>:

    exitButton: exit_button
    forwardButton: forward_button
    backwardButton: backward_button
    leftButton: left_button
    rightButton: right_button

    Button:
        id: exit_button
        font_size: 30
        size_hint: .2, .05
        center: root.width / 5, 9 * root.height / 10
        text: '<<<'
        

    Button:
        id: forward_button
        font_size: 14
        size_hint: .2, .05
        center: root.width / 2, root.height / 2
        text: 'forward'
         

    Button:
        id: backward_button
        font_size: 14
        size_hint: .2, .05
        center: root.width / 2, root.height / 2 -100
        text: 'backward'
        

    Button:
        id: left_button
        font_size: 14
        size_hint: .2, .05
        center: root.width / 2 - 50, root.height / 2 -50
        text: 'left'
        

    Button:
        id: right_button
        font_size: 14
        size_hint: .2, .05
        center: root.width / 2 + 50, root.height / 2 -50
        text: 'right'
        
""")


class HomeScreen(Screen):
    prompt = StringProperty('')


class ControlScreen(Screen):
    pass


class RemoteControlApp(App):
    connection = None

    def build(self):
        self.root = self.setup_gui()
        return self.root


    def setup_gui(self):
        self.homeScreen = HomeScreen(name='home')
        self.controlScreen = ControlScreen(name='control')
        self.homeScreen.connectButton.bind(on_press=self.connect_to_server)
        self.controlScreen.exitButton.bind(on_press=self.exit)
        self.controlScreen.forwardButton.bind(on_press=self.go_forward)
        self.controlScreen.backwardButton.bind(on_press=self.go_backward)
        self.controlScreen.leftButton.bind(on_press=self.go_left)
        self.controlScreen.rightButton.bind(on_press=self.go_right)
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
        self.print_message("Connected successfully!")
        self.connection = connection
        self.root.current = 'control'
        

    def exit(self, *args):
        self.root.current = 'home'
        self.reset()
        
    
    def go_forward(self, *args):
        if self.connection:
            self.connection.write('forward'.encode('utf-8'))
            
    
    def go_backward(self, *args):
        if self.connection:
            self.connection.write('backward'.encode('utf-8'))
            
            
    def go_left(self, *args):
        if self.connection:
            self.connection.write('left'.encode('utf-8'))
            
            
    def go_right(self, *args):
        if self.connection:
            self.connection.write('right'.encode('utf-8'))
    
    
    def print_message(self, msg):
        self.homeScreen.prompt = msg


if __name__ == '__main__':
    RemoteControlApp().run()
