from flask import Flask, render_template_string, request
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
                        <button onclick="sendCommand('arm')">ARM</button> 
                        <button onclick="sendCommand('disarm')">DISARM</button>
                        <!-- Text input and button for submission -->
                        <input type="text" id="altitude" placeholder="Altitude">
                        <button onclick="submitText()">Takeoff</button>
                        <button onclick="sendCommand('land')">Land</button>
                    </div>

                    <script>
                        // Handle text submission with onclick
                        function submitText() {
                            const altitude = document.getElementById('altitude').value;

                            fetch('/submit-text', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/x-www-form-urlencoded',
                                },
                                body: 'altitude=' + encodeURIComponent(altitude)
                            })
                            .catch(error => console.error('Error:', error));
                        }

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
            elif cmd == 'arm':
                self.event_queue.put(Event("arm_event"))
            elif cmd == 'disarm':
                self.event_queue.put(Event("disarm_event"))
            elif cmd == 'land':
                self.event_queue.put(Event("land_event"))

            # print(f"Command received: {cmd}")
            return f"Command {cmd} executed"

        # New route to handle text input submission
        @self.app.route('/submit-text', methods=['POST'])
        def submit_text():
            input_text = request.form['altitude']
            print(f"Text received: {input_text}")
            self.event_queue.put(Event("takeoff_requested_event", altitude=float(input_text)))
            return f"Text received: {input_text}"

    def run(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

    def run_in_thread(self):
        Thread(target=self.run).start()

# Example usage:
if __name__ == "__main__":
    web_page = SimpleWebPage(None)
    web_page.run_in_thread()