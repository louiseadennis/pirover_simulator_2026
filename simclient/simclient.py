"""
simclient.py provides the interface between the simulator and external python code.
Commands are sent to the simulator over UDP and sensor readings are requested only
when the matching API is called.
"""

import time
import socket
import threading


UDP_IP = "127.0.0.1"
UDP_DATA_PORT = 5000
UDP_COMMAND_PORT = 5001
PAN = 1
PUBLISH_INTERVAL = 0.02

# maximum time to wait for an on-demand sensor reply
SENSOR_TIMEOUT = 0.2


class SimulatorClient:

    def __init__(self):

        self.mutex = threading.Lock()
        self.running = True

        self.sonar_range = 1700.0
        self.sonar_angle = 0

        self.left_line_sensor_triggered = False
        self.right_line_sensor_triggered = False

        self.ir_left_triggered = False
        self.ir_middle_triggered = False
        self.ir_right_triggered = False

        self.vx = 0
        self.vth = 0

        # Pi2Go LED values
        self.front_led1_red_value = 0
        self.front_led1_green_value = 0
        self.front_led1_blue_value = 0
        self.front_led2_red_value = 0
        self.front_led2_green_value = 0
        self.front_led2_blue_value = 0

        self.left_led1_red_value = 0
        self.left_led1_green_value = 0
        self.left_led1_blue_value = 0
        self.left_led2_red_value = 0
        self.left_led2_green_value = 0
        self.left_led2_blue_value = 0

        self.right_led1_red_value = 0
        self.right_led1_green_value = 0
        self.right_led1_blue_value = 0
        self.right_led2_red_value = 0
        self.right_led2_green_value = 0
        self.right_led2_blue_value = 0

        self.back_led1_red_value = 0
        self.back_led1_green_value = 0
        self.back_led1_blue_value = 0
        self.back_led2_red_value = 0
        self.back_led2_green_value = 0
        self.back_led2_blue_value = 0

        # Pi2Go2 LED values
        self.pi2go2_leds = [[0, 0, 0] for _ in range(10)]

        # latest Pi2Go2 encoder readings
        self.left_encoder_count = 0
        self.right_encoder_count = 0

        self.fr_light_sensor = 0
        self.fl_light_sensor = 0
        self.br_light_sensor = 0
        self.bl_light_sensor = 0

        self.robot_control_switch_on = False
        self.robot_name = "Not Connected"

        # separate socket used to send sensor requests
        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # requests are numbered so the receive thread can match the reply
        self.sensor_request_id = 0
        self.sensor_events = {}
        self.sensor_results = {}
        self.sensor_lock = threading.Lock()

        self.update_thread = threading.Thread(target=self.update_state, daemon=True)
        self.update_thread.start()

        self.cmd_thread = threading.Thread(target=self.send_commands, daemon=True)
        self.cmd_thread.start()

        time.sleep(1)
        print("initialisation complete")


    def getRobotName(self):
        return self.robot_name


    def setServo(self, servo, degrees):
        """Set the angle of the panning sonar on the robot."""

        if servo == PAN:
            self.sonar_angle = degrees


    # asks the simulator to calculate one sensor and waits for the reply
    def query_sensor(self, sensor):

        with self.sensor_lock:
            self.sensor_request_id += 1
            request_id = str(self.sensor_request_id)

            event = threading.Event()
            self.sensor_events[request_id] = event

        message = f"<<SENSOR;{request_id};{sensor}>>"

        try:
            self.sensor_socket.sendto(
                message.encode("utf-8"),
                (UDP_IP, UDP_COMMAND_PORT)
            )

        except OSError:
            with self.sensor_lock:
                self.sensor_events.pop(request_id, None)
            return None

        # localhost response should normally arrive almost immediately
        if not event.wait(SENSOR_TIMEOUT):

            with self.sensor_lock:
                self.sensor_events.pop(request_id, None)
                self.sensor_results.pop(request_id, None)

            return None

        with self.sensor_lock:
            result = self.sensor_results.pop(request_id, None)
            self.sensor_events.pop(request_id, None)

        return result


    def getDistance(self):
        """Returns the current range detected by the sonar sensor."""

        value = self.query_sensor("SONAR")

        if value is not None:
            self.sonar_range = float(value)

        return self.sonar_range


    def irLeft(self):
        """Returns True if the left IR sensor detects an obstacle."""

        value = self.query_sensor("IR_LEFT")

        if value is not None:
            self.ir_left_triggered = bool(int(value))

        return self.ir_left_triggered


    def irRight(self):
        """Returns True if the right IR sensor detects an obstacle."""

        value = self.query_sensor("IR_RIGHT")

        if value is not None:
            self.ir_right_triggered = bool(int(value))

        return self.ir_right_triggered


    def irCentre(self):
        """Returns True if the centre IR sensor detects an obstacle."""

        # Initio and Pi2Go2 only have left/right obstacle sensors
        if self.robot_name.startswith("INITIO") or self.robot_name.startswith("PI2GO2"):
            return False

        value = self.query_sensor("IR_MIDDLE")

        if value is not None:
            self.ir_middle_triggered = bool(int(value))

        return self.ir_middle_triggered


    def irAll(self):
        """Returns True if any obstacle sensor is triggered."""

        if self.robot_name.startswith("INITIO") or self.robot_name.startswith("PI2GO2"):
            return self.irLeft() or self.irRight()

        return self.irLeft() or self.irRight() or self.irCentre()


    def irLeftLine(self):
        """Returns True if the left line sensor detects a line."""

        value = self.query_sensor("LINE_LEFT")

        if value is not None:
            self.left_line_sensor_triggered = bool(int(value))

        return self.left_line_sensor_triggered


    def irRightLine(self):
        """Returns True if the right line sensor detects a line."""

        value = self.query_sensor("LINE_RIGHT")

        if value is not None:
            self.right_line_sensor_triggered = bool(int(value))

        return self.right_line_sensor_triggered


     
    # Pi2Go2 wheel encoders
     

    def getEncoderLeft(self):
        """Returns the current left Pi2Go2 wheel encoder count."""

        if not self.robot_name.startswith("PI2GO2"):
            return 0

        value = self.query_sensor("ENCODER_LEFT")

        if value is not None:
            self.left_encoder_count = int(value)

        return self.left_encoder_count


    def getEncoderRight(self):
        """Returns the current right Pi2Go2 wheel encoder count."""

        if not self.robot_name.startswith("PI2GO2"):
            return 0

        value = self.query_sensor("ENCODER_RIGHT")

        if value is not None:
            self.right_encoder_count = int(value)

        return self.right_encoder_count


    def resetEncoders(self):
        """Resets both Pi2Go2 encoder counts to zero."""

        if not self.robot_name.startswith("PI2GO2"):
            return

        value = self.query_sensor("ENCODER_RESET")

        if value is not None:
            self.left_encoder_count = 0
            self.right_encoder_count = 0


    def forward(self, speed):
        """Sets both motors to move forward at speed. 0 <= speed <= 100"""

        self.vx = speed
        self.vth = 0


    def reverse(self, speed):
        """Sets both motors to reverse at speed. 0 <= speed <= 100"""

        self.vx = -speed
        self.vth = 0


    def spinLeft(self, speed):
        """Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""

        self.vx = 0
        self.vth = speed


    def spinRight(self, speed):
        """Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""

        self.vx = 0
        self.vth = -speed


    def turnForward(self, left_speed, right_speed):
        """Moves forwards in an arc by setting different speeds."""

        self.vx = left_speed + right_speed / 2.0
        self.vth = right_speed - left_speed


    def turnReverse(self, left_speed, right_speed):
        """Moves backwards in an arc by setting different speeds."""

        self.vx = -(left_speed + right_speed / 2.0)
        self.vth = right_speed - left_speed


    def stop(self):
        """Stops both motors."""

        self.vx = 0
        self.vth = 0


    def getSwitch(self):
        """Returns the value of the tact switch: True == pressed."""

        return self.robot_control_switch_on


    def getLight(self, sensor):
        """Returns 0..1023 for selected sensor, 0 <= sensor <= 3."""

        if sensor == 0:
            return self.getLightFL()
        elif sensor == 1:
            return self.getLightFR()
        elif sensor == 2:
            return self.getLightBR()
        elif sensor == 3:
            return self.getLightBL()
        else:
            return 0


    def getLightFL(self):
        """Returns 0..1023 for the front-left light sensor."""

        value = self.query_sensor("LIGHT_FL")

        if value is not None:
            self.fl_light_sensor = int(value)

        return self.fl_light_sensor


    def getLightFR(self):
        """Returns 0..1023 for the front-right light sensor."""

        value = self.query_sensor("LIGHT_FR")

        if value is not None:
            self.fr_light_sensor = int(value)

        return self.fr_light_sensor


    def getLightBL(self):
        """Returns 0..1023 for the back-left light sensor."""

        value = self.query_sensor("LIGHT_BL")

        if value is not None:
            self.bl_light_sensor = int(value)

        return self.bl_light_sensor


    def getLightBR(self):
        """Returns 0..1023 for the back-right light sensor."""

        value = self.query_sensor("LIGHT_BR")

        if value is not None:
            self.br_light_sensor = int(value)

        return self.br_light_sensor


    def cmd_vel(self, vx, vth):
        """Control the robot using linear and angular velocity."""

        self.vx = vx
        self.vth = vth


     
    # Pi2Go LEDs
     

    def setLED(self, LED, red, green, blue):
        """Sets the selected LED to the required RGB value."""

        # Pi2Go2 LEDs are individually addressable 0-9
        if self.robot_name.startswith("PI2GO2"):
            if 0 <= LED < 10:
                self.pi2go2_leds[LED] = [red, green, blue]
            return

        # original Pi2Go sets each side/pair together
        if LED == 0:
            self.front_led1_red_value = red
            self.front_led1_green_value = green
            self.front_led1_blue_value = blue
            self.front_led2_red_value = red
            self.front_led2_green_value = green
            self.front_led2_blue_value = blue

        elif LED == 1:
            self.right_led1_red_value = red
            self.right_led1_green_value = green
            self.right_led1_blue_value = blue
            self.right_led2_red_value = red
            self.right_led2_green_value = green
            self.right_led2_blue_value = blue

        elif LED == 2:
            self.back_led1_red_value = red
            self.back_led1_green_value = green
            self.back_led1_blue_value = blue
            self.back_led2_red_value = red
            self.back_led2_green_value = green
            self.back_led2_blue_value = blue

        elif LED == 3:
            self.left_led1_red_value = red
            self.left_led1_green_value = green
            self.left_led1_blue_value = blue
            self.left_led2_red_value = red
            self.left_led2_green_value = green
            self.left_led2_blue_value = blue


    def setAllLEDs(self, red, green, blue):
        """Sets all LEDs to the required RGB value."""

        if self.robot_name.startswith("PI2GO2"):
            for i in range(10):
                self.setLED(i, red, green, blue)
            return

        for i in range(4):
            self.setLED(i, red, green, blue)


    def getLED(self, LED):
        """Gets the RGB value of the selected LED."""

        if self.robot_name.startswith("PI2GO2"):
            if 0 <= LED < 10:
                return tuple(self.pi2go2_leds[LED])
            return None

        if LED == 0:
            return (
                self.front_led1_red_value,
                self.front_led1_green_value,
                self.front_led1_blue_value
            )
        elif LED == 1:
            return (
                self.front_led2_red_value,
                self.front_led2_green_value,
                self.front_led2_blue_value
            )
        elif LED == 2:
            return (
                self.right_led1_red_value,
                self.right_led1_green_value,
                self.right_led1_blue_value
            )
        elif LED == 3:
            return (
                self.right_led2_red_value,
                self.right_led2_green_value,
                self.right_led2_blue_value
            )
        elif LED == 4:
            return (
                self.back_led1_red_value,
                self.back_led1_green_value,
                self.back_led1_blue_value
            )
        elif LED == 5:
            return (
                self.back_led2_red_value,
                self.back_led2_green_value,
                self.back_led2_blue_value
            )
        elif LED == 6:
            return (
                self.left_led1_red_value,
                self.left_led1_green_value,
                self.left_led1_blue_value
            )
        elif LED == 7:
            return (
                self.left_led2_red_value,
                self.left_led2_green_value,
                self.left_led2_blue_value
            )


    def getAllLEDs(self):
        """Gets RGB values of all LEDs."""

        all_led_values = []

        led_count = 10 if self.robot_name.startswith("PI2GO2") else 8

        for i in range(led_count):
            all_led_values.append(self.getLED(i))

        return all_led_values


     
    # UDP communication
     

    def send_commands(self):
        """Continuously sends motor/LED commands to the simulator."""

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while self.running:

            try:

                if self.robot_name == "INITIO":
                    message = "<<%f;%f;%f>>" % (
                        self.vx,
                        self.vth,
                        self.sonar_angle
                    )

                    sock.sendto(
                        message.encode("utf-8"),
                        (UDP_IP, UDP_COMMAND_PORT)
                    )

                elif self.robot_name == "PI2GO2":
                    message = "<<%f;%f" % (self.vx, self.vth)
                    for led in self.pi2go2_leds:
                        message += ";%d;%d;%d" % (int(led[0]), int(led[1]), int(led[2]))
                    message += ">>"
                    sock.sendto(message.encode("utf-8"), (UDP_IP, UDP_COMMAND_PORT))

                elif self.robot_name == "PI2GO":
                    message = "<<%f;%f;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d>>" % (
                        self.vx,
                        self.vth,
                        int(self.front_led1_red_value),
                        int(self.front_led1_green_value),
                        int(self.front_led1_blue_value),
                        int(self.front_led2_red_value),
                        int(self.front_led2_green_value),
                        int(self.front_led2_blue_value),
                        int(self.right_led1_red_value),
                        int(self.right_led1_green_value),
                        int(self.right_led1_blue_value),
                        int(self.right_led2_red_value),
                        int(self.right_led2_green_value),
                        int(self.right_led2_blue_value),
                        int(self.back_led1_red_value),
                        int(self.back_led1_green_value),
                        int(self.back_led1_blue_value),
                        int(self.back_led2_red_value),
                        int(self.back_led2_green_value),
                        int(self.back_led2_blue_value),
                        int(self.left_led1_red_value),
                        int(self.left_led1_green_value),
                        int(self.left_led1_blue_value),
                        int(self.left_led2_red_value),
                        int(self.left_led2_green_value),
                        int(self.left_led2_blue_value)
                    )

                    sock.sendto(
                        message.encode("utf-8"),
                        (UDP_IP, UDP_COMMAND_PORT)
                    )

                time.sleep(PUBLISH_INTERVAL)

            except OSError:
                self.running = False

        sock.close()
        print("closed send socket\n")


    def update_state(self):
        """Receives normal state packets and immediate sensor replies."""

        print("starting update thread")
        sock = None

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((UDP_IP, UDP_DATA_PORT))

        except OSError:
            print("Could not open socket - have you cleaned up last connection?\n")
            self.running = False

        while self.running and sock is not None:

            try:
                data_e, _ = sock.recvfrom(1024)

            except OSError:
                break

            data = data_e.decode()

            if not data.startswith("<<") or not data.endswith(">>"):
                continue

            data = data[2:-2]
            values_list = data.split(";")

            # reply from an on-demand sensor request
            if len(values_list) == 4 and values_list[0] == "SENSOR":

                request_id = values_list[1]
                result = values_list[3]

                with self.sensor_lock:
                    self.sensor_results[request_id] = result
                    event = self.sensor_events.get(request_id)

                    if event is not None:
                        event.set()

                continue

            # normal state packet
            self.robot_name = str(values_list[0])

            if self.robot_name.startswith("INITIO") and len(values_list) >= 11:
                self.sonar_range = float(values_list[1])
                self.left_line_sensor_triggered = int(values_list[2])
                self.right_line_sensor_triggered = int(values_list[3])
                self.ir_left_triggered = int(values_list[4])
                self.ir_right_triggered = int(values_list[5])
                self.fl_light_sensor = int(values_list[6])
                self.fr_light_sensor = int(values_list[7])
                self.br_light_sensor = int(values_list[8])
                self.bl_light_sensor = int(values_list[9])
                self.robot_control_switch_on = int(values_list[10])

            elif self.robot_name == "PI2GO2" and len(values_list) >= 42:
                self.sonar_range = float(values_list[1])
                self.left_line_sensor_triggered = int(values_list[2])
                self.right_line_sensor_triggered = int(values_list[3])
                self.ir_left_triggered = int(values_list[4])
                self.ir_middle_triggered = False
                self.ir_right_triggered = int(values_list[6])
                self.fl_light_sensor = int(values_list[7])
                self.fr_light_sensor = int(values_list[8])
                self.br_light_sensor = int(values_list[9])
                self.bl_light_sensor = int(values_list[10])
                self.robot_control_switch_on = int(values_list[41])

            elif self.robot_name == "PI2GO" and len(values_list) >= 36:
                self.sonar_range = float(values_list[1])
                self.left_line_sensor_triggered = int(values_list[2])
                self.right_line_sensor_triggered = int(values_list[3])
                self.ir_left_triggered = int(values_list[4])
                self.ir_middle_triggered = int(values_list[5])
                self.ir_right_triggered = int(values_list[6])
                self.fl_light_sensor = int(values_list[7])
                self.fr_light_sensor = int(values_list[8])
                self.br_light_sensor = int(values_list[9])
                self.bl_light_sensor = int(values_list[10])
                self.robot_control_switch_on = int(values_list[35])

        if sock is not None:
            sock.close()

        print("closed update socket\n")


    def cleanup(self):

        self.running = False

        # stop any new sensor requests
        try:
            self.sensor_socket.close()
        except OSError:
            pass

        # release a request if cleanup happens while it is waiting
        with self.sensor_lock:
            for event in self.sensor_events.values():
                event.set()

            self.sensor_events.clear()
            self.sensor_results.clear()
