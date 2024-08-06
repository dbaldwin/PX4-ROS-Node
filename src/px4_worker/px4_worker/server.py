from flask import Flask, render_template_string
from threading import Thread
from pysm import Event
class SimpleWebPage:
    def __init__(self, event_queue):
        self.app = Flask(__name__)
        self.setup_routes()
        self.event_queue = event_queue

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string('''
                <html>
                <head>
                    <title>Control Page</title>
                    <style>
                        .button-container {
                            display: flex;
                            flex-direction: column;
                            align-items: center;
                            gap: 10px;
                            margin-top: 50px;
                        }
                        button {
                            width: 100px;
                            height: 50px;
                        }
                    </style>
                </head>
                <body>
                    <div class="button-container">
                        <button onclick="sendCommand('up')">Up</button>
                        <button onclick="sendCommand('down')">Down</button>
                        <button onclick="sendCommand('forwards')">Forwards</button>
                        <button onclick="sendCommand('left')">Left</button>
                        <button onclick="sendCommand('right')">Right</button>
                        <button onclick="sendCommand('backwards')">Backwards</button>
                        <button onclick="sendCommand('yaw_left')">Turn Left</button>
                        <button onclick="sendCommand('yaw_right')">Turn Right</button>                 
                    </div>

                    <script>
                        function sendCommand(command) {
                            fetch('/command/' + command, {method: 'POST'})
                            .then(response => response.text())
                            .then(data => console.log(data));
                        }
                    </script>
                </body>
                </html>
            ''')

        @self.app.route('/command/<cmd>', methods=['POST'])
        def command(cmd):
            # print(f"Command received: {cmd}")
            # print(f"cmd:type {type(cmd)}")
            if cmd == 'up':
                self.event_queue.put(Event("up_event"))
            elif cmd == 'down':
                self.event_queue.put(Event("down_event"))
            elif cmd == 'left':
                self.event_queue.put(Event("left_event"))
            elif cmd == 'right':
                self.event_queue.put(Event("right_event"))
            elif cmd == 'forwards':
                self.event_queue.put(Event("forward_event"))
            elif cmd == 'backwards':
                self.event_queue.put(Event("backward_event"))
            elif cmd == 'yaw_left':
                self.event_queue.put(Event("yaw_left_event"))
            elif cmd == 'yaw_right':
                self.event_queue.put(Event("yaw_right_event"))

            # print(f"Command received: {cmd}")
            return f"Command {cmd} executed"

    def run(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

    def run_in_thread(self):
        Thread(target=self.run).start()

# Example usage:
if __name__ == "__main__":
    web_page = SimpleWebPage(None)
    web_page.run_in_thread()