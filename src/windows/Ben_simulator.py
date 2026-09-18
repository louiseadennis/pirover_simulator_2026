from pathlib import Path

import math
import random
import socket
import threading
import time
import xml.etree.ElementTree as ET

from tkinter import filedialog, messagebox, simpledialog

import pyglet

# disables shadow window - potential reason for old code breaking
pyglet.options["shadow_window"] = False

from pyglet.window import key, mouse

from src.windows.new_objectwindow import ObjectWindow


# robot display size (pixels)
ROBOT_WIDTH = 70

# defaults for WASD
FORWARD_SPEED = 150
TURNING_SPEED = 120


# approx positions of the virtual line sensors
LINE_SENSOR_FORWARD = 40
LINE_SENSOR_SIDE = 14

# upper RGB brightness to trigger custom line sensor
LINE_DARKNESS_THRESHOLD = 100

# standard deviation added to custom line brightness readings
LINE_BRIGHTNESS_NOISE = 5.0

# default false positive/negative percentages
DEFAULT_FALSE_POSITIVE_PERCENT = 2.0
DEFAULT_FALSE_NEGATIVE_PERCENT = 2.0


# sonar position/range
SONAR_OFFSET_X = 50
SONAR_MIN_RANGE = 5
SONAR_MAX_RANGE = 1700

# sonar uses a few rays to approximate a beam
SONAR_BEAM_ANGLE = 0.36
SONAR_RAYS = 5

# standard deviation of sonar distance noise
SONAR_DISTANCE_NOISE = 3.0


# IR sensor positions/range
IR_SENSOR_ANGLE = 0.785
IR_OFFSET_X = 33
IR_OFFSET_X_MIDDLE = 52
IR_OFFSET_Y = 21

IR_MIN_RANGE = 5
IR_MAX_RANGE = 35

# smaller beam than sonar
IR_BEAM_ANGLE = 0.25
IR_RAYS = 3

# standard deviation used around IR detection distance
IR_DISTANCE_NOISE = 2.0


# robot LED display
LED_RADIUS = 2.8
LED_OUTLINE_RADIUS = 3.8
LED_OFF_COLOR = (35, 35, 35)


# Pi2Go2 encoder setup
PI2GO2_WHEEL_DIAMETER_MM = 65.0
PI2GO2_ENCODER_PULSES_PER_REV = 20
PI2GO2_ROBOT_WIDTH_MM = 118.0

# use robot width as wheel spacing until the actual spacing is measured
PI2GO2_WHEEL_TRACK_MM = 118.0
PI2GO2_MM_PER_PIXEL = PI2GO2_ROBOT_WIDTH_MM / ROBOT_WIDTH


# delay added to each sensor type when latency is enabled (seconds)
SONAR_LATENCY = 0.08
IR_LATENCY = 0.02
LINE_LATENCY = 0.01
LIGHT_LATENCY = 0.03
ENCODER_LATENCY = 0.005


# maximum/minimum light sensor output
LIGHT_MAX_VALUE = 1023
LIGHT_MIN_VALUE = 10

# light is treated as full strength when this close to a sensor
LIGHT_FULL_DISTANCE = 50

# controls how wide each light sensor's field of view is
LIGHT_STDDEV = math.pi / 3

# standard deviation added to the final light sensor reading
LIGHT_VALUE_NOISE = 8.0


# simulator/simclient UDP connection
UDP_IP = "127.0.0.1"
UDP_DATA_PORT = 5000
UDP_COMMAND_PORT = 5001
SOCKET_INTERVAL = 0.03


# moves image anchor to midpoint
def centre_image(image):
    image.anchor_x = image.width // 2
    image.anchor_y = image.height // 2


# Simulator:
#   load XML world file
#   draw the world and robot
#   move the robot
#   collision checks
#   edit world
#   simulate sensors when queried
#   UDP communication with simclient
#   save edited worlds
class Simulator(pyglet.window.Window):

    def __init__(
        self,
        selected_file,
        selected_robot,
        start_window
    ):

        # tkinter choices
        self.selected_file = selected_file
        self.selected_robot = selected_robot

        # reference to original tkinter window
        self.start_window = start_window


        # Robot-Workshop folder
        self.repo_folder = (
            Path(__file__)
            .resolve()
            .parent
            .parent
            .parent
        )


        # original world path - saving never writes back to this file
        self.xml_path = (
            self.repo_folder
            / "worlds"
            / selected_file
        )


        # read selected XML
        # after this, edits are made to temporary pyglet objects in memory
        tree = ET.parse(self.xml_path)
        self.world = tree.getroot()


        self.world_width = int(
            self.world.get("width")
        )

        self.world_height = int(
            self.world.get("height")
        )

        self.background_index = int(
            self.world.get("background_index")
        )

        self.sonar_resolution = int(
            self.world.get(
                "sonar_resolution",
                "10"
            )
        )


        # robot starting position/orientation
        robot = self.world.find("robot")

        self.robot_x = float(
            robot.get("position_x")
        )

        self.robot_y = float(
            robot.get("position_y")
        )

        self.robot_rotation = float(
            robot.get("rotation")
        )


        # create main simulator window
        super().__init__(
            width=self.world_width,
            height=self.world_height,
            caption="Robot Workshop",
        )
        
        self.current_scale = self.scale
        self.set_size(self.world_width/self.current_scale, self.world_height/self.current_scale)

        # activate simulator OpenGL context
        self.switch_to()


        # tracks if runtime world differs from original file
        self.unsaved_changes = False

        # program state flags
        self.initialising = True
        self.world_closing = False


        # background sprite
        self.background = None


        # line map image, pixels and sprite
        self.line_image = None
        self.line_data = None
        self.line_pixels = None
        self.line_sprite = None

        # identifies original map0-map9 or custom map
        self.line_map_type = None
        self.line_map_index = None
        self.custom_line_filename = None


        # normal obstacles and light source
        self.static_objects = []


        # editor state
        self.object_window = None
        self.edit_mode = False

        # object currently being moved
        self.drag_target = None

        # transparent object shown while dragging from toolbar
        self.toolbar_preview = None
        self.preview_type = None
        self.preview_index = None


        # UDP sockets
        self.command_socket = None
        self.state_socket = None

        # latest movement command from simclient
        self.socket_vx = 0
        self.socket_vth = 0

        self.socket_running = True


        # Initio sonar can rotate using its servo
        self.sonar_angle = 0


        # Gaussian sensor noise can be turned on/off from the simulator
        self.sensor_noise_enabled = True

        # false positive/negative errors are separate from Gaussian noise
        self.false_positives_enabled = False

        # user-set percentages, both default to 2%
        self.false_positive_percent = DEFAULT_FALSE_POSITIVE_PERCENT
        self.false_negative_percent = DEFAULT_FALSE_NEGATIVE_PERCENT

        # optional delay before a requested sensor value is returned
        self.sensor_latency_enabled = False


        # LED colours sent by simclient
        self.robot_led_values = []
        self.robot_leds = []
        self.robot_led_outlines = []
        self.robot_led_offsets = []


        # Pi2Go2 encoder counts
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_encoder_fraction = 0.0
        self.right_encoder_fraction = 0.0


        # latest sensor readings
        # these only change when the sensor is queried
        self.sonar_distance = SONAR_MAX_RANGE

        self.left_line_triggered = False
        self.right_line_triggered = False

        self.ir_left_triggered = False
        self.ir_middle_triggered = False
        self.ir_right_triggered = False

        self.light_fl = 0
        self.light_fr = 0
        self.light_br = 0
        self.light_bl = 0


        # create world from XML/resources
        self.load_background()
        self.load_line_map()
        self.load_object_images()
        self.load_objects()
        self.load_lights()
        self.load_robot()
        self.load_robot_leds()
        self.load_menu_buttons()


        # records keys currently being held down
        self.keys = key.KeyStateHandler()
        self.push_handlers(self.keys)


        # UDP runs separately so waiting for data cannot freeze pyglet
        self.command_thread = threading.Thread(
            target=self.receive_commands,
            daemon=True
        )

        self.state_thread = threading.Thread(
            target=self.send_state,
            daemon=True
        )

        self.command_thread.start()
        self.state_thread.start()


        # everything above belongs to original loaded world
        self.initialising = False
        self.unsaved_changes = False

        self.update_caption()

    def on_move(self, x, y):
        if self.scale != self.current_scale:
            self.current_scale = self.scale
            
            self.set_size(
                int(self.world_width/self.scale),
                int(self.world_height/self.scale)
            )
        
     
    # world loading
     

    def load_background(self):

        self.set_background(
            self.background_index,
            dirty=False
        )


    def set_background(self, index, dirty=True):

        # background needs simulator OpenGL context
        self.switch_to()

        image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "backgrounds"
                / f"bg{index}.png"
            )
        )

        sprite = pyglet.sprite.Sprite(
            image,
            x=0,
            y=0
        )

        # stretch image to fill world
        sprite.scale_x = (
            self.world_width
            / image.width
        )

        sprite.scale_y = (
            self.world_height
            / image.height
        )


        if self.background is not None:
            self.background.delete()

        self.background = sprite
        self.background_index = index


        if dirty:
            self.mark_dirty()


    def load_line_map(self):

        # original worlds use map0-map9
        line_map = self.world.find("line_map")

        if line_map is not None:

            index = int(
                line_map.get("index")
            )

            # some worlds use -1 for no line map
            if index >= 0:

                self.set_line_map(
                    index,
                    float(
                        line_map.get("position_x")
                    ),
                    float(
                        line_map.get("position_y")
                    ),
                    dirty=False
                )

                return


        # otherwise check for custom line map
        custom = self.world.find(
            "custom_line_map"
        )

        if custom is not None:

            filename = custom.get(
                "filename"
            )

            if filename:

                self.set_custom_line_map(
                    filename,
                    dirty=False
                )


    def set_line_map(
        self,
        index,
        x,
        y,
        dirty=True
    ):

        self.switch_to()

        image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "line_maps"
                / f"map{index}.png"
            )
        )

        centre_image(image)


        # only one active line map
        if self.line_sprite is not None:
            self.line_sprite.delete()


        self.line_image = image
        self.line_data = image.get_image_data()

        # copy pixels into normal memory once
        # avoids repeatedly asking pyglet for image regions
        self.line_pixels = self.line_data.get_data(
            "RGBA",
            self.line_data.width * 4
        )


        self.line_sprite = pyglet.sprite.Sprite(
            image,
            x=x,
            y=y
        )


        self.line_map_type = "original"
        self.line_map_index = index
        self.custom_line_filename = None


        if dirty:
            self.mark_dirty()


    def set_custom_line_map(
        self,
        filename,
        dirty=True
    ):

        self.switch_to()

        image = pyglet.image.load(
            str(
                self.repo_folder
                / filename
            )
        )


        if self.line_sprite is not None:
            self.line_sprite.delete()


        self.line_image = image
        self.line_data = image.get_image_data()

        # cache pixel data for fast line sensor checks
        self.line_pixels = self.line_data.get_data(
            "RGBA",
            self.line_data.width * 4
        )


        self.line_sprite = pyglet.sprite.Sprite(
            image,
            x=0,
            y=0
        )

        # custom line image covers whole world
        self.line_sprite.scale_x = (
            self.world_width
            / image.width
        )

        self.line_sprite.scale_y = (
            self.world_height
            / image.height
        )


        self.line_map_type = "custom"
        self.line_map_index = None
        self.custom_line_filename = filename


        if dirty:
            self.mark_dirty()


    def remove_line_map(self):

        self.switch_to()

        if self.line_sprite is not None:
            self.line_sprite.delete()


        self.line_image = None
        self.line_data = None
        self.line_pixels = None
        self.line_sprite = None

        self.line_map_type = None
        self.line_map_index = None
        self.custom_line_filename = None


        self.mark_dirty()


    def load_object_images(self):

        self.switch_to()

        # original object sprite sheet
        sheet = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "static_objects"
                / "boxesv2.png"
            )
        )

        # split into 9 separate object images
        self.object_images = list(
            pyglet.image.ImageGrid(
                sheet,
                1,
                9
            )
        )

        for image in self.object_images:
            centre_image(image)


    def make_object(self, index, x, y):

        self.switch_to()

        image = self.object_images[index]

        sprite = pyglet.sprite.Sprite(
            image,
            x=x,
            y=y
        )

        # stored so object can later be written back into XML
        sprite.idx = index
        sprite.object_type = "object"

        return sprite


    def load_objects(self):

        # load every static object from XML
        for obj in self.world.findall(
            "static_object"
        ):

            index = int(
                obj.get("index")
            )

            if index < 0:
                continue


            sprite = self.make_object(
                index,
                float(
                    obj.get("position_x")
                ),
                float(
                    obj.get("position_y")
                )
            )

            self.static_objects.append(
                sprite
            )


    def make_light(self, x, y):

        self.switch_to()

        image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "static_objects"
                / "light.png"
            )
        )

        centre_image(image)


        sprite = pyglet.sprite.Sprite(
            image,
            x=x,
            y=y
        )

        # distinguishes light from normal obstacles
        sprite.idx = -1
        sprite.object_type = "light"

        return sprite


    def load_lights(self):

        for light in self.world.findall(
            "light_source"
        ):

            sprite = self.make_light(
                float(
                    light.get("position_x")
                ),
                float(
                    light.get("position_y")
                )
            )

            self.static_objects.append(
                sprite
            )


    def load_robot(self):

        self.switch_to()

        # select robot image from tkinter choice
        if self.selected_robot == "Initio":
            filename = "rover.png"

        elif self.selected_robot == "Pi2Go2":
            filename = "pi2go2.png"

        else:
            filename = "pi2go.png"


        image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "robot"
                / filename
            )
        )

        centre_image(image)


        self.robot = pyglet.sprite.Sprite(
            image,
            x=self.robot_x,
            y=self.robot_y
        )

        # same displayed width for both robots
        self.robot.scale = (
            ROBOT_WIDTH
            / image.width
        )

        # pyglet rotation runs opposite to simulation heading
        self.heading = -self.robot_rotation
        self.robot.rotation = self.robot_rotation


     
    # robot LEDs
     

    def load_robot_leds(self):

        # Initio does not use the Pi2Go LED display
        if self.selected_robot == "Initio":
            return

        half_width = self.robot.width / 2
        half_height = self.robot.height / 2

        if self.selected_robot == "Pi2Go2":

            # first 8 LEDs run across the front
            front_x = half_width - 5
            top_y = half_height - 6
            bottom_y = -half_height + 6

            front_leds = []
            for i in range(8):
                y = top_y + (bottom_y - top_y) * i / 7
                front_leds.append((front_x - 10, y))

            # final 2 LEDs sit at the rear corners
            back_x = -half_width + 6
            self.robot_led_offsets = front_leds + [
                (back_x, half_height - 7),
                (back_x, -half_height + 7)
            ]

        else:

            # Pi2Go packet order is front, right, back, left
            front_x = half_width - 5
            back_x = -half_width + 6
            side_y = half_height - 4

            # about 2/3 of the way from the rear to the front
            side_x = -half_width + self.robot.width * 2 / 3

            self.robot_led_offsets = [
                # front pair
                (front_x - 11, half_height - 16),
                (front_x - 11, -half_height + 16),

                # right pair
                (side_x - 5, -side_y + 12),
                (side_x - 14, -side_y + 12),

                # back pair
                (back_x + 2, half_height - 17),
                (back_x + 2, -half_height + 17),

                # left pair
                (side_x - 5, side_y - 12),
                (side_x - 14 , side_y - 12)
            ]

        self.robot_led_values = [
            [0, 0, 0]
            for _ in self.robot_led_offsets
        ]

        # each LED is a dark outline with the coloured LED on top
        for _ in self.robot_led_offsets:
            outline = pyglet.shapes.Circle(
                self.robot.x,
                self.robot.y,
                LED_OUTLINE_RADIUS,
                color=(15, 15, 15)
            )

            led = pyglet.shapes.Circle(
                self.robot.x,
                self.robot.y,
                LED_RADIUS,
                color=LED_OFF_COLOR
            )

            self.robot_led_outlines.append(outline)
            self.robot_leds.append(led)


    def set_robot_led_values(self, values, led_count):

        # RGB values start after vx and vth
        for i in range(led_count):
            start = 2 + i * 3

            try:
                red = int(float(values[start]))
                green = int(float(values[start + 1]))
                blue = int(float(values[start + 2]))
            except (ValueError, IndexError):
                continue

            self.robot_led_values[i] = [
                max(0, min(255, red)),
                max(0, min(255, green)),
                max(0, min(255, blue))
            ]


    def update_robot_led_display(self):

        if not self.robot_leds:
            return

        angle = math.radians(self.heading)
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)

        for i, (offset_x, offset_y) in enumerate(self.robot_led_offsets):

            # rotate LED position with the robot
            x = self.robot.x + offset_x * cos_angle - offset_y * sin_angle
            y = self.robot.y + offset_x * sin_angle + offset_y * cos_angle

            self.robot_led_outlines[i].x = x
            self.robot_led_outlines[i].y = y
            self.robot_leds[i].x = x
            self.robot_leds[i].y = y

            red, green, blue = self.robot_led_values[i]

            if red == 0 and green == 0 and blue == 0:
                self.robot_leds[i].color = LED_OFF_COLOR
            else:
                self.robot_leds[i].color = (red, green, blue)


    def draw_robot_leds(self):

        self.update_robot_led_display()

        for outline, led in zip(self.robot_led_outlines, self.robot_leds):
            outline.draw()
            led.draw()


    def load_menu_buttons(self):

        self.switch_to()


        # edit/save/close buttons all load in same way
        def load_button(filename, x):

            image = pyglet.image.load(
                str(
                    self.repo_folder
                    / "resources"
                    / "menu_buttons"
                    / filename
                )
            )

            centre_image(image)


            sprite = pyglet.sprite.Sprite(
                image,
                x=x,
                y=self.height - 25
            )

            sprite.scale_x = 50 / image.width
            sprite.scale_y = 25 / image.height

            return sprite


        self.edit_button = load_button(
            "button_edit.png",
            50
        )

        self.save_button = load_button(
            "button_save.png",
            100
        )

        self.close_button = load_button(
            "button_close.png",
            150
        )


        # noise toggle does not need another image file
        noise_image = (
            pyglet.image.SolidColorImagePattern(
                (90, 90, 90, 255)
            )
            .create_image(
                70,
                25
            )
        )

        centre_image(noise_image)

        self.noise_button = pyglet.sprite.Sprite(
            noise_image,
            x=215,
            y=self.height - 25
        )

        self.noise_button_label = pyglet.text.Label(
            "NOISE ON",
            x=215,
            y=self.height - 25,
            anchor_x="center",
            anchor_y="center",
            font_size=9
        )


        # false positive/negative toggle is kept separate from Gaussian noise
        false_positive_image = (
            pyglet.image.SolidColorImagePattern(
                (90, 90, 90, 255)
            )
            .create_image(
                90,
                25
            )
        )

        centre_image(false_positive_image)

        self.false_positive_button = pyglet.sprite.Sprite(
            false_positive_image,
            x=300,
            y=self.height - 25
        )

        self.false_positive_button_label = pyglet.text.Label(
            "FALSE +/- OFF",
            x=300,
            y=self.height - 25,
            anchor_x="center",
            anchor_y="center",
            font_size=8
        )


        # click to change false positive and false negative percentages
        false_percent_image = (
            pyglet.image.SolidColorImagePattern(
                (90, 90, 90, 255)
            )
            .create_image(
                70,
                25
            )
        )

        centre_image(false_percent_image)

        self.false_percent_button = pyglet.sprite.Sprite(
            false_percent_image,
            x=385,
            y=self.height - 25
        )

        self.false_percent_button_label = pyglet.text.Label(
            "+2% -2%",
            x=385,
            y=self.height - 25,
            anchor_x="center",
            anchor_y="center",
            font_size=8
        )


        # latency only delays requested sensor replies
        latency_image = (
            pyglet.image.SolidColorImagePattern(
                (90, 90, 90, 255)
            )
            .create_image(
                80,
                25
            )
        )

        centre_image(latency_image)

        self.latency_button = pyglet.sprite.Sprite(
            latency_image,
            x=465,
            y=self.height - 25
        )

        self.latency_button_label = pyglet.text.Label(
            "LATENCY OFF",
            x=465,
            y=self.height - 25,
            anchor_x="center",
            anchor_y="center",
            font_size=8
        )


     
    # sensor noise
     

    def toggle_sensor_noise(self):

        self.sensor_noise_enabled = (
            not self.sensor_noise_enabled
        )

        if self.sensor_noise_enabled:
            self.noise_button_label.text = "NOISE ON"

        else:
            self.noise_button_label.text = "NOISE OFF"

        print(
            "Sensor noise:",
            "ON" if self.sensor_noise_enabled else "OFF"
        )


     
    # false positive/negative readings
     

    def toggle_false_positives(self):

        self.false_positives_enabled = (
            not self.false_positives_enabled
        )

        if self.false_positives_enabled:
            self.false_positive_button_label.text = "FALSE +/- ON"

        else:
            self.false_positive_button_label.text = "FALSE +/- OFF"

        print(
            "False positive/negative readings:",
            "ON" if self.false_positives_enabled else "OFF"
        )


    def set_false_reading_percentages(self):

        parent = self.start_window.window

        false_positive = simpledialog.askfloat(
            "False positives",
            "False positive percentage (0-100):",
            initialvalue=self.false_positive_percent,
            minvalue=0.0,
            maxvalue=100.0,
            parent=parent
        )

        if false_positive is None:
            return

        false_negative = simpledialog.askfloat(
            "False negatives",
            "False negative percentage (0-100):",
            initialvalue=self.false_negative_percent,
            minvalue=0.0,
            maxvalue=100.0,
            parent=parent
        )

        if false_negative is None:
            return

        self.false_positive_percent = false_positive
        self.false_negative_percent = false_negative

        self.false_percent_button_label.text = (
            f"+{false_positive:g}% -{false_negative:g}%"
        )

        print(
            "False positive/negative percentages:",
            f"{false_positive:g}% / {false_negative:g}%"
        )


     
    # sensor latency
     

    def toggle_sensor_latency(self):

        self.sensor_latency_enabled = (
            not self.sensor_latency_enabled
        )

        if self.sensor_latency_enabled:
            self.latency_button_label.text = "LATENCY ON"

        else:
            self.latency_button_label.text = "LATENCY OFF"

        print(
            "Sensor latency:",
            "ON" if self.sensor_latency_enabled else "OFF"
        )


    def get_sensor_latency(self, sensor):

        # each sensor type has its own response delay
        if sensor == "SONAR":
            return SONAR_LATENCY

        if sensor.startswith("IR_"):
            return IR_LATENCY

        if sensor.startswith("LINE_"):
            return LINE_LATENCY

        if sensor.startswith("LIGHT_"):
            return LIGHT_LATENCY

        if sensor.startswith("ENCODER_"):
            return ENCODER_LATENCY

        return 0


     
    # editor
     

    def mark_dirty(self):

        # loading original world should not count as an edit
        if self.initialising:
            return

        self.unsaved_changes = True
        self.update_caption()


    def update_caption(self):

        # * shows world has unsaved changes
        marker = (
            " *"
            if self.unsaved_changes
            else ""
        )

        self.set_caption(
            f"Robot Workshop - "
            f"{self.selected_file}"
            f"{marker}"
        )


    def edit(self):

        # close editor if already open
        if self.edit_mode:

            self.edit_mode = False
            self.drag_target = None

            self.clear_preview()


            if self.object_window is not None:

                window = self.object_window
                self.object_window = None

                window.close()

                # return OpenGL context to simulator
                self.switch_to()

            return


        # otherwise open toolbar
        self.edit_mode = True

        self.object_window = ObjectWindow(
            240,
            self.height,
            self.repo_folder,
            self
        )


        # put editor immediately to right of simulator
        x, y = self.get_location()

        self.object_window.set_location(
            x + self.width,
            y
        )

        self.object_window.set_visible(True)
        self.object_window.activate()


        # creating toolbar changes OpenGL context
        self.switch_to()


    def preview_tool(
        self,
        tool,
        index,
        x,
        y,
        inside
    ):

        # function is normally called from toolbar window
        self.switch_to()


        # background/delete do not need a preview sprite
        if (
            not inside
            or tool in (
                "background",
                "delete"
            )
        ):

            self.clear_preview()
            return


        # only recreate preview when selected tool changes
        if (
            self.toolbar_preview is None
            or self.preview_type != tool
            or self.preview_index != index
        ):

            self.clear_preview()


            if tool == "object":

                image = self.object_images[
                    index
                ]


            elif tool == "line_map":

                image = pyglet.image.load(
                    str(
                        self.repo_folder
                        / "resources"
                        / "line_maps"
                        / f"map{index}.png"
                    )
                )

                centre_image(image)


            elif tool == "light":

                image = pyglet.image.load(
                    str(
                        self.repo_folder
                        / "resources"
                        / "static_objects"
                        / "light.png"
                    )
                )

                centre_image(image)


            else:
                return


            self.toolbar_preview = (
                pyglet.sprite.Sprite(
                    image,
                    x=x,
                    y=y
                )
            )

            # shows that object has not yet been placed
            self.toolbar_preview.opacity = 150

            self.preview_type = tool
            self.preview_index = index


        # existing preview just follows mouse
        self.toolbar_preview.x = x
        self.toolbar_preview.y = y


    def clear_preview(self):

        self.switch_to()

        if self.toolbar_preview is not None:
            self.toolbar_preview.delete()

        self.toolbar_preview = None
        self.preview_type = None
        self.preview_index = None


    def add_tool(
        self,
        tool,
        index,
        x,
        y
    ):

        # usually called from editor window
        self.switch_to()


        if tool == "object":

            self.static_objects.append(
                self.make_object(
                    index,
                    x,
                    y
                )
            )

            self.mark_dirty()


        elif tool == "line_map":

            self.set_line_map(
                index,
                x,
                y
            )


        elif tool == "background":

            self.set_background(
                index
            )


        elif tool == "delete":

            self.delete_at(
                x,
                y
            )


        elif tool == "light":

            # currently only one light source is supported
            for light in list(
                self.static_objects
            ):

                if getattr(
                    light,
                    "object_type",
                    ""
                ) == "light":

                    self.static_objects.remove(
                        light
                    )

                    light.delete()


            self.static_objects.append(
                self.make_light(
                    x,
                    y
                )
            )

            self.mark_dirty()


    def selected_tool(self):

        # used by original right-click placement
        if self.object_window is None:
            return "none", -1

        return (
            self.object_window
            .get_selected_sprite_name()
        )


    def add_selected_tool(self, x, y):

        tool, index = self.selected_tool()

        self.add_tool(
            tool,
            index,
            x,
            y
        )


    def delete_at(self, x, y):

        self.switch_to()

        # latest/topmost object is checked first
        for sprite in reversed(
            self.static_objects
        ):

            if self.mouse_over(
                sprite,
                x,
                y
            ):

                self.static_objects.remove(
                    sprite
                )

                sprite.delete()

                self.mark_dirty()
                return


        # line map can also be deleted
        if (
            self.line_sprite is not None
            and self.mouse_over(
                self.line_sprite,
                x,
                y
            )
        ):

            self.remove_line_map()


    def find_drag_target(self, x, y):

        # normal objects first
        for sprite in reversed(
            self.static_objects
        ):

            if self.mouse_over(
                sprite,
                x,
                y
            ):

                return sprite


        # original line maps can be moved
        if (
            self.line_map_type == "original"
            and self.line_sprite is not None
            and self.mouse_over(
                self.line_sprite,
                x,
                y
            )
        ):

            return self.line_sprite


        # robot can also be repositioned
        if self.mouse_over(
            self.robot,
            x,
            y
        ):

            return self.robot


        return None


     
    # line sensors
     

    def get_line_sensor_positions(self):

        angle = math.radians(
            self.heading
        )

        # direction robot is facing
        forward_x = math.cos(angle)
        forward_y = math.sin(angle)

        # direction 90 degrees to robot's left
        left_x = -math.sin(angle)
        left_y = math.cos(angle)


        # midpoint between front sensors
        front_x = (
            self.robot.x
            + LINE_SENSOR_FORWARD
            * forward_x
        )

        front_y = (
            self.robot.y
            + LINE_SENSOR_FORWARD
            * forward_y
        )


        # offset one sensor each side
        left = (
            front_x
            + LINE_SENSOR_SIDE
            * left_x,

            front_y
            + LINE_SENSOR_SIDE
            * left_y
        )

        right = (
            front_x
            - LINE_SENSOR_SIDE
            * left_x,

            front_y
            - LINE_SENSOR_SIDE
            * left_y
        )


        return left, right


    def position_is_on_line(
        self,
        world_x,
        world_y
    ):

        if self.line_pixels is None:
            return False


        if self.line_map_type == "original":

            # original map uses its centre as position
            map_left = (
                self.line_sprite.x
                - self.line_image.width / 2
            )

            map_bottom = (
                self.line_sprite.y
                - self.line_image.height / 2
            )


            # world coordinate to image pixel
            image_x = int(
                world_x - map_left
            )

            image_y = int(
                world_y - map_bottom
            )


        elif self.line_map_type == "custom":

            # custom map fills whole simulation window
            image_x = int(
                world_x
                / self.world_width
                * self.line_data.width
            )

            image_y = int(
                world_y
                / self.world_height
                * self.line_data.height
            )


        else:
            return False


        # point is outside image
        if (
            image_x < 0
            or image_y < 0
            or image_x >= self.line_data.width
            or image_y >= self.line_data.height
        ):

            return False


        # direct location of pixel in cached RGBA data
        position = (
            image_y
            * self.line_data.width
            + image_x
        ) * 4


        red = self.line_pixels[
            position
        ]

        green = self.line_pixels[
            position + 1
        ]

        blue = self.line_pixels[
            position + 2
        ]

        alpha = self.line_pixels[
            position + 3
        ]


        # original maps use transparency
        if self.line_map_type == "original":
            return alpha > 0


        # ignore transparent custom pixels
        if alpha == 0:
            return False


        # custom map uses dark pixels as line
        brightness = (
            red
            + green
            + blue
        ) / 3

        # add noise before applying the threshold when enabled
        if self.sensor_noise_enabled:

            measured_brightness = (
                brightness
                + random.gauss(
                    0,
                    LINE_BRIGHTNESS_NOISE
                )
            )

        else:
            measured_brightness = brightness


        return (
            measured_brightness
            < LINE_DARKNESS_THRESHOLD
        )


    def calculate_line_sensor(self, side):

        # only calculate requested line sensor
        left, right = (
            self.get_line_sensor_positions()
        )


        if side == "left":

            triggered = (
                self.position_is_on_line(
                    *left
                )
            )

            # occasionally miss a real line or report one that is not there
            if (
                triggered
                and self.false_positives_enabled
                and random.random() < (self.false_negative_percent / 100.0)
            ):
                triggered = False

            elif (
                not triggered
                and self.false_positives_enabled
                and random.random() < (self.false_positive_percent / 100.0)
            ):
                triggered = True

            self.left_line_triggered = triggered

            return self.left_line_triggered


        if side == "right":

            triggered = (
                self.position_is_on_line(
                    *right
                )
            )

            # occasionally miss a real line or report one that is not there
            if (
                triggered
                and self.false_positives_enabled
                and random.random() < (self.false_negative_percent / 100.0)
            ):
                triggered = False

            elif (
                not triggered
                and self.false_positives_enabled
                and random.random() < (self.false_positive_percent / 100.0)
            ):
                triggered = True

            self.right_line_triggered = triggered

            return self.right_line_triggered


        return False


     
    # sonar / IR
     

    # converts a position relative to robot into world coordinates
    def get_sensor_position(
        self,
        offset_x,
        offset_y
    ):

        angle = math.radians(
            self.heading
        )


        x = (
            self.robot.x
            + offset_x
            * math.cos(angle)
            - offset_y
            * math.sin(angle)
        )

        y = (
            self.robot.y
            + offset_x
            * math.sin(angle)
            + offset_y
            * math.cos(angle)
        )


        return x, y


    # finds where a ray first enters an obstacle rectangle
    # this avoids checking every pixel along the sensor beam
    def ray_box_distance(
        self,
        start_x,
        start_y,
        direction_x,
        direction_y,
        sprite,
        max_range
    ):

        left = (
            sprite.x
            - sprite.width / 2
        )

        right = (
            sprite.x
            + sprite.width / 2
        )

        bottom = (
            sprite.y
            - sprite.height / 2
        )

        top = (
            sprite.y
            + sprite.height / 2
        )


        nearest = 0
        furthest = max_range


        # intersection with left/right side
        if abs(direction_x) < 0.000001:

            if (
                start_x < left
                or start_x > right
            ):

                return None


        else:

            t1 = (
                left - start_x
            ) / direction_x

            t2 = (
                right - start_x
            ) / direction_x


            nearest = max(
                nearest,
                min(t1, t2)
            )

            furthest = min(
                furthest,
                max(t1, t2)
            )


        # intersection with top/bottom side
        if abs(direction_y) < 0.000001:

            if (
                start_y < bottom
                or start_y > top
            ):

                return None


        else:

            t1 = (
                bottom - start_y
            ) / direction_y

            t2 = (
                top - start_y
            ) / direction_y


            nearest = max(
                nearest,
                min(t1, t2)
            )

            furthest = min(
                furthest,
                max(t1, t2)
            )


        # no valid intersection
        if (
            nearest > furthest
            or furthest < 0
        ):

            return None


        if (
            0 <= nearest <= max_range
        ):

            return nearest


        return None


    # finds distance from ray to edge of world
    def ray_to_world_edge(
        self,
        start_x,
        start_y,
        direction_x,
        direction_y
    ):

        distances = []


        if direction_x > 0:

            distances.append(
                (
                    self.width
                    - start_x
                )
                / direction_x
            )


        elif direction_x < 0:

            distances.append(
                (
                    0 - start_x
                )
                / direction_x
            )


        if direction_y > 0:

            distances.append(
                (
                    self.height
                    - start_y
                )
                / direction_y
            )


        elif direction_y < 0:

            distances.append(
                (
                    0 - start_y
                )
                / direction_y
            )


        distances = [
            distance
            for distance in distances
            if distance >= 0
        ]


        if not distances:
            return SONAR_MAX_RANGE


        return min(distances)


    # casts one sensor ray through current world
    def cast_sensor_ray(
        self,
        start_x,
        start_y,
        angle,
        max_range
    ):

        direction_x = math.cos(angle)
        direction_y = math.sin(angle)


        # world boundary also acts as obstacle
        nearest = min(
            max_range,
            self.ray_to_world_edge(
                start_x,
                start_y,
                direction_x,
                direction_y
            )
        )


        # copy allows editor to alter object list without affecting loop
        for sprite in list(
            self.static_objects
        ):

            # light is not a physical obstacle
            if getattr(
                sprite,
                "object_type",
                ""
            ) == "light":

                continue


            distance = (
                self.ray_box_distance(
                    start_x,
                    start_y,
                    direction_x,
                    direction_y,
                    sprite,
                    max_range
                )
            )


            if (
                distance is not None
                and distance < nearest
            ):

                nearest = distance


        return min(
            nearest,
            max_range
        )


    def calculate_sonar(self):

        # sonar mounted at front centre
        start_x, start_y = (
            self.get_sensor_position(
                SONAR_OFFSET_X,
                0
            )
        )


        # Initio sonar can also rotate using servo
        angle = (
            math.radians(
                self.heading
                + self.sonar_angle
            )
        )


        nearest = SONAR_MAX_RANGE


        # several rays approximate sonar cone
        for i in range(
            SONAR_RAYS
        ):

            if SONAR_RAYS == 1:
                offset = 0

            else:

                offset = (
                    i
                    / (
                        SONAR_RAYS
                        - 1
                    )
                    - 0.5
                ) * SONAR_BEAM_ANGLE


            distance = (
                self.cast_sensor_ray(
                    start_x,
                    start_y,
                    angle + offset,
                    SONAR_MAX_RANGE
                )
            )


            if distance < nearest:
                nearest = distance


        # add measurement noise after finding true distance when enabled
        if self.sensor_noise_enabled:

            measured_distance = (
                nearest
                + random.gauss(
                    0,
                    SONAR_DISTANCE_NOISE
                )
            )

        else:
            measured_distance = nearest

        # keep reading inside the physical sensor range
        self.sonar_distance = max(
            SONAR_MIN_RANGE,
            min(
                SONAR_MAX_RANGE,
                measured_distance
            )
        )


        return round(
            self.sonar_distance,
            2
        )


    def calculate_ir(self, sensor):

        # position/direction differs for each IR sensor
        if sensor == "left":

            offset_x = IR_OFFSET_X
            offset_y = IR_OFFSET_Y
            angle_offset = IR_SENSOR_ANGLE


        elif sensor == "middle":

            offset_x = IR_OFFSET_X_MIDDLE
            offset_y = 0
            angle_offset = 0


        elif sensor == "right":

            offset_x = IR_OFFSET_X
            offset_y = -IR_OFFSET_Y
            angle_offset = -IR_SENSOR_ANGLE


        else:
            return False


        start_x, start_y = (
            self.get_sensor_position(
                offset_x,
                offset_y
            )
        )


        angle = (
            math.radians(
                self.heading
            )
            + angle_offset
        )


        # with noise on, check slightly beyond the normal range so an
        # object near the boundary can sometimes be measured inside it
        if self.sensor_noise_enabled:

            check_range = (
                IR_MAX_RANGE
                + 4 * IR_DISTANCE_NOISE
            )

        else:
            check_range = IR_MAX_RANGE

        nearest = check_range


        # three rays approximate IR detection cone
        for i in range(
            IR_RAYS
        ):

            if IR_RAYS == 1:
                offset = 0

            else:

                offset = (
                    i
                    / (
                        IR_RAYS
                        - 1
                    )
                    - 0.5
                ) * IR_BEAM_ANGLE


            distance = (
                self.cast_sensor_ray(
                    start_x,
                    start_y,
                    angle + offset,
                    check_range
                )
            )


            if distance < nearest:
                nearest = distance


        # add noise before deciding if the IR sensor triggers
        if self.sensor_noise_enabled:

            measured_distance = (
                nearest
                + random.gauss(
                    0,
                    IR_DISTANCE_NOISE
                )
            )

        else:
            measured_distance = nearest

        triggered = (
            IR_MIN_RANGE
            <= measured_distance
            < IR_MAX_RANGE
        )

        # occasionally miss a real obstacle or report one that is not there
        if (
            triggered
            and self.false_positives_enabled
            and random.random() < (self.false_negative_percent / 100.0)
        ):
            triggered = False

        elif (
            not triggered
            and self.false_positives_enabled
            and random.random() < (self.false_positive_percent / 100.0)
        ):
            triggered = True


        if sensor == "left":
            self.ir_left_triggered = triggered

        elif sensor == "middle":
            self.ir_middle_triggered = triggered

        elif sensor == "right":
            self.ir_right_triggered = triggered


        return triggered


     
    # light sensors
     

    def get_light_source(self):

        # currently only one light source is supported
        for sprite in self.static_objects:

            if getattr(
                sprite,
                "object_type",
                ""
            ) == "light":

                return sprite


        return None


    # smallest angle between two directions
    def angle_difference(
        self,
        angle_a,
        angle_b
    ):

        return abs(
            (
                angle_a
                - angle_b
                + math.pi
            )
            % (
                2 * math.pi
            )
            - math.pi
        )


    # checks if the robot or an obstacle sits between sensor and light
    def light_is_blocked(
        self,
        sensor_x,
        sensor_y,
        sensor_angle,
        light
    ):

        # start just outside the robot so the sensor does not
        # immediately detect the edge it is mounted on
        start_x = (
            sensor_x
            + math.cos(sensor_angle)
        )

        start_y = (
            sensor_y
            + math.sin(sensor_angle)
        )


        dx = light.x - start_x
        dy = light.y - start_y

        distance_to_light = math.hypot(
            dx,
            dy
        )


        if distance_to_light == 0:
            return False


        # unit vector from sensor towards light
        direction_x = dx / distance_to_light
        direction_y = dy / distance_to_light


        # check if light would have to pass through the robot
        distance = self.ray_box_distance(
            start_x,
            start_y,
            direction_x,
            direction_y,
            self.robot,
            distance_to_light
        )


        if (
            distance is not None
            and distance < distance_to_light
        ):

            return True


        # check normal world obstacles
        for sprite in self.static_objects:

            # light does not block itself
            if getattr(
                sprite,
                "object_type",
                ""
            ) == "light":

                continue


            distance = self.ray_box_distance(
                start_x,
                start_y,
                direction_x,
                direction_y,
                sprite,
                distance_to_light
            )


            if (
                distance is not None
                and distance < distance_to_light
            ):

                return True


        return False


    def calculate_light(self, sensor):

        light = self.get_light_source()


        if light is None:

            value = 0


        else:

            half_width = (
                self.robot.width / 2
            )

            half_height = (
                self.robot.height / 2
            )


            # approximate location of four Pi2Go light sensors
            offsets = {

                "fl": (
                    half_width,
                    half_height - 7
                ),

                "fr": (
                    half_width,
                    -half_height + 7
                ),

                "br": (
                    -half_width,
                    -half_height + 7
                ),

                "bl": (
                    -half_width,
                    half_height - 7
                )
            }


            if sensor not in offsets:
                return 0


            offset_x, offset_y = (
                offsets[sensor]
            )


            sensor_x, sensor_y = (
                self.get_sensor_position(
                    offset_x,
                    offset_y
                )
            )


            # direction this light sensor faces
            sensor_angle = math.atan2(
                sensor_y - self.robot.y,
                sensor_x - self.robot.x
            )


            # direction and distance from sensor to light
            dx = light.x - sensor_x
            dy = light.y - sensor_y

            light_angle = math.atan2(
                dy,
                dx
            )

            distance = math.hypot(
                dx,
                dy
            )


            # no light reaches the sensor if something is in the way
            if self.light_is_blocked(
                sensor_x,
                sensor_y,
                sensor_angle,
                light
            ):

                value = 0


            else:

                difference = (
                    self.angle_difference(
                        sensor_angle,
                        light_angle
                    )
                )


                # light radiates in all directions but each sensor
                # has a broad directional sensitivity
                direction_intensity = math.exp(
                    -(
                        difference ** 2
                    )
                    / (
                        2
                        * LIGHT_STDDEV ** 2
                    )
                )


                # maximum possible distance across the current world
                max_distance = math.hypot(
                    self.world_width,
                    self.world_height
                )

                max_distance = max(
                    LIGHT_FULL_DISTANCE + 1,
                    max_distance
                )


                # close to the light gives the full 1023 reading
                if distance <= LIGHT_FULL_DISTANCE:

                    distance_value = LIGHT_MAX_VALUE


                else:

                    # inverse-square fall-off, normalised so a nearby
                    # light gives 1023 and the furthest distance gives 10
                    distance = min(
                        distance,
                        max_distance
                    )

                    current_strength = 1 / (
                        distance ** 2
                    )

                    near_strength = 1 / (
                        LIGHT_FULL_DISTANCE ** 2
                    )

                    far_strength = 1 / (
                        max_distance ** 2
                    )


                    distance_intensity = (
                        current_strength
                        - far_strength
                    ) / (
                        near_strength
                        - far_strength
                    )


                    distance_value = (
                        LIGHT_MIN_VALUE
                        + distance_intensity
                        * (
                            LIGHT_MAX_VALUE
                            - LIGHT_MIN_VALUE
                        )
                    )


                # combine distance with the direction the sensor faces
                value = (
                    distance_value
                    * direction_intensity
                )

                # add noise after the ideal light value is calculated
                if self.sensor_noise_enabled:

                    value += random.gauss(
                        0,
                        LIGHT_VALUE_NOISE
                    )

                # light sensor output is always kept between 0 and 1023
                value = int(
                    max(
                        0,
                        min(
                            LIGHT_MAX_VALUE,
                            value
                        )
                    )
                )


        if sensor == "fl":
            self.light_fl = value

        elif sensor == "fr":
            self.light_fr = value

        elif sensor == "br":
            self.light_br = value

        elif sensor == "bl":
            self.light_bl = value


        return value


     
    # collision
     

    def mouse_over(
        self,
        sprite,
        x,
        y
    ):

        # rectangular mouse hitbox
        return (
            sprite.x
            - sprite.width / 2
            <= x
            <= sprite.x
            + sprite.width / 2

            and

            sprite.y
            - sprite.height / 2
            <= y
            <= sprite.y
            + sprite.height / 2
        )


    def sprites_collide(
        self,
        a,
        b
    ):

        # simple rectangular collision
        return (
            a.x
            + a.width / 2
            > b.x
            - b.width / 2

            and

            a.x
            - a.width / 2
            < b.x
            + b.width / 2

            and

            a.y
            + a.height / 2
            > b.y
            - b.height / 2

            and

            a.y
            - a.height / 2
            < b.y
            + b.height / 2
        )


    def robot_hits_obstacle(self):

        for sprite in self.static_objects:

            # light is not a physical obstacle
            if getattr(
                sprite,
                "object_type",
                ""
            ) == "light":

                continue


            if self.sprites_collide(
                self.robot,
                sprite
            ):

                return True


        return False


    def keep_robot_in_window(self):

        half_width = (
            self.robot.width / 2
        )

        half_height = (
            self.robot.height / 2
        )


        self.robot.x = max(
            half_width,
            min(
                self.width
                - half_width,
                self.robot.x
            )
        )


        self.robot.y = max(
            half_height,
            min(
                self.height
                - half_height,
                self.robot.y
            )
        )


     
    # mouse / keyboard
     

    def on_mouse_press(
        self,
        x,
        y,
        button,
        modifiers
    ):

        if button == mouse.LEFT:


            if self.mouse_over(
                self.edit_button,
                x,
                y
            ):

                self.edit()
                return


            if self.mouse_over(
                self.save_button,
                x,
                y
            ):

                self.save_world()
                return


            if self.mouse_over(
                self.close_button,
                x,
                y
            ):

                self.close_world()
                return


            if self.mouse_over(
                self.noise_button,
                x,
                y
            ):

                self.toggle_sensor_noise()
                return


            if self.mouse_over(
                self.false_positive_button,
                x,
                y
            ):

                self.toggle_false_positives()
                return


            if self.mouse_over(
                self.false_percent_button,
                x,
                y
            ):

                self.set_false_reading_percentages()
                return


            if self.mouse_over(
                self.latency_button,
                x,
                y
            ):

                self.toggle_sensor_latency()
                return


            # start dragging existing object
            if self.edit_mode:

                self.drag_target = (
                    self.find_drag_target(
                        x,
                        y
                    )
                )

                return


        # original right-click placement method
        if (
            button == mouse.RIGHT
            and self.edit_mode
        ):

            self.add_selected_tool(
                x,
                y
            )


    def on_mouse_drag(
        self,
        x,
        y,
        dx,
        dy,
        buttons,
        modifiers
    ):

        if (
            not self.edit_mode
            or self.drag_target is None
            or not buttons & mouse.LEFT
        ):

            return


        self.drag_target.x += dx
        self.drag_target.y += dy

        # only runtime world has changed
        self.mark_dirty()


    def on_mouse_release(
        self,
        x,
        y,
        button,
        modifiers
    ):

        if button == mouse.LEFT:
            self.drag_target = None


    def on_key_press(
        self,
        symbol,
        modifiers
    ):

        # E toggles editor
        if symbol == key.E:
            self.edit()


        # ctrl + S opens Save As
        elif (
            symbol == key.S
            and modifiers & key.MOD_CTRL
        ):

            self.save_world()


        # Q closes current world
        elif symbol == key.Q:
            self.close_world()


     
    # saving
     

    def build_world_xml(self):

        # creates XML from current runtime world
        # nothing is written to disk here
        root = ET.Element(
            "world",
            {
                "background_index":
                    str(
                        self.background_index
                    ),

                "width":
                    str(
                        self.world_width
                    ),

                "height":
                    str(
                        self.world_height
                    ),

                "sonar_resolution":
                    str(
                        self.sonar_resolution
                    )
            }
        )


        # current robot position/orientation
        ET.SubElement(
            root,
            "robot",
            {
                "position_x":
                    str(
                        round(
                            self.robot.x
                        )
                    ),

                "position_y":
                    str(
                        round(
                            self.robot.y
                        )
                    ),

                "rotation":
                    str(
                        round(
                            self.robot.rotation
                        )
                    )
            }
        )


        # original line map
        if (
            self.line_map_type == "original"
            and self.line_sprite is not None
        ):

            ET.SubElement(
                root,
                "line_map",
                {
                    "index":
                        str(
                            self.line_map_index
                        ),

                    "position_x":
                        str(
                            round(
                                self.line_sprite.x
                            )
                        ),

                    "position_y":
                        str(
                            round(
                                self.line_sprite.y
                            )
                        )
                }
            )


        # custom full-world line map
        elif (
            self.line_map_type == "custom"
            and self.custom_line_filename
        ):

            ET.SubElement(
                root,
                "custom_line_map",
                {
                    "filename":
                        self.custom_line_filename
                }
            )


        # save all current obstacles/lights
        for sprite in self.static_objects:

            if getattr(
                sprite,
                "object_type",
                ""
            ) == "light":

                ET.SubElement(
                    root,
                    "light_source",
                    {
                        "position_x":
                            str(
                                round(
                                    sprite.x
                                )
                            ),

                        "position_y":
                            str(
                                round(
                                    sprite.y
                                )
                            )
                    }
                )


            else:

                ET.SubElement(
                    root,
                    "static_object",
                    {
                        "index":
                            str(
                                sprite.idx
                            ),

                        "position_x":
                            str(
                                round(
                                    sprite.x
                                )
                            ),

                        "position_y":
                            str(
                                round(
                                    sprite.y
                                )
                            )
                    }
                )


        return ET.ElementTree(
            root
        )


    def save_world(self):

        tree = self.build_world_xml()

        # keeps saved XML readable
        ET.indent(
            tree,
            space="    "
        )


        worlds_folder = (
            self.repo_folder
            / "worlds"
        )

        suggested_name = (
            f"{self.xml_path.stem}"
            f"_edited.xml"
        )


        while True:

            filename = (
                filedialog
                .asksaveasfilename(
                    parent=
                        self.start_window.window,

                    title=
                        "Save World As",

                    initialdir=
                        str(
                            worlds_folder
                        ),

                    initialfile=
                        suggested_name,

                    defaultextension=
                        ".xml",

                    filetypes=[
                        (
                            "Robot Workshop World",
                            "*.xml"
                        )
                    ]
                )
            )


            # cancel returns to simulator
            if not filename:

                self.switch_to()
                self.activate()

                return


            save_path = Path(
                filename
            )


            # original file cannot be overwritten
            if (
                save_path.resolve()
                == self.xml_path.resolve()
            ):

                messagebox.showwarning(
                    "Choose a new file",
                    (
                        "The original world "
                        "cannot be overwritten.\n\n"
                        "Please choose a new filename."
                    ),
                    parent=
                        self.start_window.window
                )

                continue


            try:

                tree.write(
                    save_path,
                    encoding="utf-8",
                    xml_declaration=True
                )


            except OSError as error:

                messagebox.showerror(
                    "Save failed",
                    str(error),
                    parent=
                        self.start_window.window
                )

                continue


            self.unsaved_changes = False
            self.update_caption()

            print(
                "World saved as:",
                save_path
            )


            # tkinter dialog changes focus/context
            self.switch_to()
            self.activate()

            return


     
    # Pi2Go2 wheel encoders
     

    def update_encoders(self, linear_speed, angular_speed, dt):

        if self.selected_robot != "Pi2Go2" or dt <= 0:
            return

        # simulator speed is in pixels/sec, encoders use the real wheel size
        linear_speed_mm = linear_speed * PI2GO2_MM_PER_PIXEL
        angular_speed_rad = math.radians(angular_speed)

        # work out how fast each wheel is moving
        left_speed_mm = linear_speed_mm - angular_speed_rad * PI2GO2_WHEEL_TRACK_MM / 2
        right_speed_mm = linear_speed_mm + angular_speed_rad * PI2GO2_WHEEL_TRACK_MM / 2

        wheel_circumference = math.pi * PI2GO2_WHEEL_DIAMETER_MM

        # distance travelled -> wheel turns -> encoder pulses
        left_pulses = left_speed_mm * dt / wheel_circumference * PI2GO2_ENCODER_PULSES_PER_REV
        right_pulses = right_speed_mm * dt / wheel_circumference * PI2GO2_ENCODER_PULSES_PER_REV

        # keep part pulses until they make a full encoder pulse
        self.left_encoder_fraction += left_pulses
        self.right_encoder_fraction += right_pulses

        left_whole = math.trunc(self.left_encoder_fraction)
        right_whole = math.trunc(self.right_encoder_fraction)

        self.left_encoder_count += left_whole
        self.right_encoder_count += right_whole
        self.left_encoder_fraction -= left_whole
        self.right_encoder_fraction -= right_whole


     
    # sensor requests
     

    def handle_sensor_request(
        self,
        values,
        sock
    ):

        # expected packet:
        # <<SENSOR;request_id;sensor_name>>
        if len(values) != 3:
            return


        request_id = values[1]
        sensor = values[2]


        if sensor == "SONAR":

            value = (
                self.calculate_sonar()
            )


        elif sensor == "LINE_LEFT":

            value = int(
                self.calculate_line_sensor(
                    "left"
                )
            )


        elif sensor == "LINE_RIGHT":

            value = int(
                self.calculate_line_sensor(
                    "right"
                )
            )


        elif sensor == "IR_LEFT":

            value = int(
                self.calculate_ir(
                    "left"
                )
            )


        elif sensor == "IR_MIDDLE":

            # no middle obstacle sensor on Pi2Go2
            if self.selected_robot == "Pi2Go2":
                value = 0
            else:
                value = int(self.calculate_ir("middle"))


        elif sensor == "IR_RIGHT":

            value = int(
                self.calculate_ir(
                    "right"
                )
            )


        elif sensor == "LIGHT_FL":

            value = (
                self.calculate_light(
                    "fl"
                )
            )


        elif sensor == "LIGHT_FR":

            value = (
                self.calculate_light(
                    "fr"
                )
            )


        elif sensor == "LIGHT_BR":

            value = (
                self.calculate_light(
                    "br"
                )
            )


        elif sensor == "LIGHT_BL":

            value = (
                self.calculate_light(
                    "bl"
                )
            )


        elif sensor == "ENCODER_LEFT":
            value = self.left_encoder_count if self.selected_robot == "Pi2Go2" else 0


        elif sensor == "ENCODER_RIGHT":
            value = self.right_encoder_count if self.selected_robot == "Pi2Go2" else 0


        elif sensor == "ENCODER_RESET":

            if self.selected_robot == "Pi2Go2":
                self.left_encoder_count = 0
                self.right_encoder_count = 0
                self.left_encoder_fraction = 0.0
                self.right_encoder_fraction = 0.0

            value = 1


        else:
            return


        # response does not wait for the normal state publishing loop
        message = (
            f"<<SENSOR;"
            f"{request_id};"
            f"{sensor};"
            f"{value}>>"
        )


        def send_response():

            try:

                sock.sendto(
                    message.encode(
                        "utf-8"
                    ),
                    (
                        UDP_IP,
                        UDP_DATA_PORT
                    )
                )

            except OSError:
                pass


        # timer adds latency without holding up movement commands
        if self.sensor_latency_enabled:

            timer = threading.Timer(
                self.get_sensor_latency(sensor),
                send_response
            )
            timer.daemon = True
            timer.start()

        else:
            send_response()


     
    # UDP communication
     

    def receive_commands(self):

        # receives movement/sensor requests from simclient
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.command_socket = sock


        try:

            sock.bind(
                (
                    UDP_IP,
                    UDP_COMMAND_PORT
                )
            )

            # allows thread to check regularly if world closed
            sock.settimeout(
                0.5
            )

            print(
                "Listening for commands on port",
                UDP_COMMAND_PORT
            )


            while self.socket_running:

                try:

                    data, _ = (
                        sock.recvfrom(
                            1024
                        )
                    )


                except socket.timeout:
                    continue


                except OSError:
                    break


                message = data.decode(
                    "utf-8"
                )


                # valid packets use << >>
                if (
                    not message.startswith("<<")
                    or not message.endswith(">>")
                ):

                    continue


                values = (
                    message[2:-2]
                    .split(";")
                )


                # sensor requests are answered immediately
                if (
                    len(values) > 0
                    and values[0] == "SENSOR"
                ):

                    self.handle_sensor_request(
                        values,
                        sock
                    )

                    continue


                # Pi2Go2 command packet
                if len(values) == 32:

                    self.socket_vx = float(values[0])
                    self.socket_vth = float(values[1])
                    self.set_robot_led_values(values, 10)


                # Pi2Go command packet
                elif len(values) == 26:

                    self.socket_vx = float(
                        values[0]
                    )

                    self.socket_vth = float(
                        values[1]
                    )

                    self.set_robot_led_values(values, 8)


                # Initio command packet
                elif len(values) == 3:

                    self.socket_vx = float(
                        values[0]
                    )

                    self.socket_vth = float(
                        values[1]
                    )

                    # third value controls sonar servo
                    self.sonar_angle = float(
                        values[2]
                    )


        except OSError as error:

            if self.socket_running:

                print(
                    "Could not open command socket:",
                    error
                )


        finally:

            try:
                sock.close()

            except OSError:
                pass


            if self.command_socket is sock:
                self.command_socket = None


    def send_state(self):

        # normal state packets are still sent for backwards compatibility
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.state_socket = sock


        print(
            "Publishing state on port",
            UDP_DATA_PORT
        )


        try:

            while self.socket_running:

                # use correct robot identifier
                if self.selected_robot == "Initio":
                    robot_name = "INITIO"

                elif self.selected_robot == "Pi2Go2":
                    robot_name = "PI2GO2"

                else:
                    robot_name = "PI2GO"


                # these are cached sensor values
                # no sensor calculation happens here
                ir_middle = self.ir_middle_triggered
                if self.selected_robot == "Pi2Go2":
                    ir_middle = False

                values = [

                    robot_name,

                    str(
                        self.sonar_distance
                    ),

                    str(
                        int(
                            self.left_line_triggered
                        )
                    ),

                    str(
                        int(
                            self.right_line_triggered
                        )
                    ),

                    str(
                        int(
                            self.ir_left_triggered
                        )
                    ),

                    str(
                        int(
                            ir_middle
                        )
                    ),

                    str(
                        int(
                            self.ir_right_triggered
                        )
                    ),

                    str(
                        self.light_fl
                    ),

                    str(
                        self.light_fr
                    ),

                    str(
                        self.light_br
                    ),

                    str(
                        self.light_bl
                    )
                ]


                # keep the same packet layout and send the LED colours too
                if self.selected_robot == "Pi2Go2":
                    for led in self.robot_led_values:
                        values += [str(led[0]), str(led[1]), str(led[2])]

                else:
                    for led in self.robot_led_values:
                        values += [str(led[0]), str(led[1]), str(led[2])]

                    # Initio has no robot LEDs but keeps the old packet length
                    if self.selected_robot == "Initio":
                        values += ["0"] * 24

                # control switch
                values.append("1")


                message = (
                    "<<"
                    + ";".join(
                        values
                    )
                    + ">>"
                )


                try:

                    sock.sendto(
                        message.encode(
                            "utf-8"
                        ),
                        (
                            UDP_IP,
                            UDP_DATA_PORT
                        )
                    )


                except OSError:
                    break


                time.sleep(
                    SOCKET_INTERVAL
                )


        finally:

            try:
                sock.close()

            except OSError:
                pass


            if self.state_socket is sock:
                self.state_socket = None


    def stop_sockets(self):

        # tells both UDP threads to finish
        self.socket_running = False

        self.socket_vx = 0
        self.socket_vth = 0


        for sock in (
            self.command_socket,
            self.state_socket
        ):

            if sock is not None:

                try:
                    sock.close()

                except OSError:
                    pass


        self.command_socket = None
        self.state_socket = None


     
    # main simulation
     

    def update(self, dt):

        # toolbar may be closed separately using its X
        if (
            self.object_window is not None
            and self.object_window.close_me
        ):

            self.object_window = None
            self.edit_mode = False
            self.drag_target = None

            self.clear_preview()


        # normal movement comes from simclient
        linear_speed = self.socket_vx
        angular_speed = self.socket_vth


        # WASD retained for simple testing
        if self.keys[key.W]:

            linear_speed = (
                FORWARD_SPEED
            )


        elif self.keys[key.S]:

            linear_speed = (
                -FORWARD_SPEED
            )


        if self.keys[key.A]:

            angular_speed = (
                TURNING_SPEED
            )


        elif self.keys[key.D]:

            angular_speed = (
                -TURNING_SPEED
            )


        # Pi2Go2 encoders count while the robot is moving
        self.update_encoders(linear_speed, angular_speed, dt)


        # update robot orientation
        self.heading += (
            angular_speed
            * dt
        )

        self.robot.rotation = (
            -self.heading
        )


        # turn speed/heading into x/y movement
        angle = math.radians(
            self.heading
        )

        dx = (
            linear_speed
            * math.cos(angle)
            * dt
        )

        dy = (
            linear_speed
            * math.sin(angle)
            * dt
        )


        # try horizontal movement
        old_x = self.robot.x
        self.robot.x += dx

        if self.robot_hits_obstacle():
            self.robot.x = old_x


        # try vertical movement
        old_y = self.robot.y
        self.robot.y += dy

        if self.robot_hits_obstacle():
            self.robot.y = old_y


        self.keep_robot_in_window()

        # sensors are deliberately NOT calculated here
        # they are calculated only when simrobot queries them


    def on_draw(self):

        self.clear()

        # draw world from back to front
        self.background.draw()


        if self.line_sprite is not None:
            self.line_sprite.draw()


        for sprite in self.static_objects:
            sprite.draw()


        self.robot.draw()
        self.draw_robot_leds()


        # toolbar drag preview
        if self.toolbar_preview is not None:
            self.toolbar_preview.draw()


        # buttons drawn last
        self.edit_button.draw()
        self.save_button.draw()
        self.close_button.draw()
        self.noise_button.draw()
        self.noise_button_label.draw()
        self.false_positive_button.draw()
        self.false_positive_button_label.draw()
        self.false_percent_button.draw()
        self.false_percent_button_label.draw()
        self.latency_button.draw()
        self.latency_button_label.draw()


     
    # closing
     

    def cleanup(self):

        # avoids cleanup being run twice
        if self.world_closing:
            return


        self.world_closing = True


        # stop UDP threads
        self.stop_sockets()

        self.drag_target = None


        # remove preview before OpenGL context disappears
        try:

            self.switch_to()

            if self.toolbar_preview is not None:
                self.toolbar_preview.delete()


        except Exception:
            pass


        self.toolbar_preview = None


        # close editor window
        if self.object_window is not None:

            window = self.object_window

            self.object_window = None
            self.edit_mode = False


            try:
                window.close()

            except Exception:
                pass


        # runtime edits are discarded if they were not saved
        if self.unsaved_changes:

            print(
                "World closed - "
                "unsaved edits discarded."
            )


    def close_world(self):

        # close current world but leave tkinter running
        self.cleanup()

        self.close()

        # returns control to pysim.py
        pyglet.app.exit()


    def on_close(self):

        # window X behaves same as CLOSE button
        self.cleanup()

        pyglet.app.exit()

        return super().on_close()
