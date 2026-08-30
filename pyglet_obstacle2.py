import math
from pathlib import Path

import pyglet
from pyglet.window import key, mouse


# --------------------------------------------------
# CREATE WINDOW
# --------------------------------------------------

window = pyglet.window.Window(
    width=800,
    height=600,
    caption="Robot World",
)


# --------------------------------------------------
# FIND IMAGE FILES
# --------------------------------------------------

folder = Path(__file__).resolve().parent

robot_image_path = folder / "pi2go.png"
obstacle_image_path = folder / "box.png"


if not robot_image_path.exists():
    raise FileNotFoundError(
        f"Robot image not found: {robot_image_path}"
    )

if not obstacle_image_path.exists():
    raise FileNotFoundError(
        f"Obstacle image not found: {obstacle_image_path}"
    )


# --------------------------------------------------
# LOAD ROBOT IMAGE
# --------------------------------------------------

robot_image = pyglet.image.load(
    str(robot_image_path)
)

# Make centre of image the position/rotation point.
robot_image.anchor_x = robot_image.width // 2
robot_image.anchor_y = robot_image.height // 2


# --------------------------------------------------
# CREATE ROBOT
# --------------------------------------------------

robot = pyglet.sprite.Sprite(
    img=robot_image,
    x=150,
    y=300,
)

# Display robot approximately 70 pixels wide.
robot.scale = 70 / robot_image.width


# --------------------------------------------------
# LOAD OBSTACLE IMAGE
# --------------------------------------------------

obstacle_image = pyglet.image.load(
    str(obstacle_image_path)
)

obstacle_image.anchor_x = obstacle_image.width // 2
obstacle_image.anchor_y = obstacle_image.height // 2


# --------------------------------------------------
# OBSTACLE CLASS
# --------------------------------------------------

class Obstacle:

    def __init__(self, x, y):

        self.sprite = pyglet.sprite.Sprite(
            img=obstacle_image,
            x=x,
            y=y,
        )

        # Set obstacle size.
        self.sprite.scale = 40 / obstacle_image.width

    def draw(self):

        self.sprite.draw()


# --------------------------------------------------
# STORE ALL OBSTACLES
# --------------------------------------------------

obstacles = []


# --------------------------------------------------
# ROBOT MOVEMENT SETTINGS
# --------------------------------------------------

heading = 0.0

FORWARD_SPEED = 150
TURNING_SPEED = 120


# Record keys currently held down.
keys = key.KeyStateHandler()
window.push_handlers(keys)


# --------------------------------------------------
# COLLISION BETWEEN TWO SPRITES
# --------------------------------------------------

def sprites_collide(sprite1, sprite2):

    # Robot boundaries.
    left1 = sprite1.x - sprite1.width / 2
    right1 = sprite1.x + sprite1.width / 2
    bottom1 = sprite1.y - sprite1.height / 2
    top1 = sprite1.y + sprite1.height / 2

    # Obstacle boundaries.
    left2 = sprite2.x - sprite2.width / 2
    right2 = sprite2.x + sprite2.width / 2
    bottom2 = sprite2.y - sprite2.height / 2
    top2 = sprite2.y + sprite2.height / 2

    return (
        right1 > left2
        and left1 < right2
        and top1 > bottom2
        and bottom1 < top2
    )


# --------------------------------------------------
# CHECK ALL OBSTACLES
# --------------------------------------------------

def robot_hits_obstacle():

    for obstacle in obstacles:

        if sprites_collide(
            robot,
            obstacle.sprite
        ):
            return True

    return False


# --------------------------------------------------
# KEEP ROBOT INSIDE WINDOW
# --------------------------------------------------

def keep_robot_in_window():

    half_width = robot.width / 2
    half_height = robot.height / 2

    robot.x = max(
        half_width,
        min(
            window.width - half_width,
            robot.x,
        ),
    )

    robot.y = max(
        half_height,
        min(
            window.height - half_height,
            robot.y,
        ),
    )


# --------------------------------------------------
# PLACE OBSTACLES WITH MOUSE
# --------------------------------------------------

@window.event
def on_mouse_press(x, y, button, modifiers):

    if button == mouse.LEFT:

        new_obstacle = Obstacle(
            x,
            y
        )

        obstacles.append(
            new_obstacle
        )

        print(
            f"Obstacle placed at ({x}, {y})"
        )


# --------------------------------------------------
# UPDATE ROBOT
# --------------------------------------------------

def update(dt):

    global heading

    linear_speed = 0.0
    angular_speed = 0.0


    # Forward / backwards
    if keys[key.W]:
        linear_speed = FORWARD_SPEED

    elif keys[key.S]:
        linear_speed = -FORWARD_SPEED


    # Rotate
    if keys[key.A]:
        angular_speed = TURNING_SPEED

    elif keys[key.D]:
        angular_speed = -TURNING_SPEED


    # Update robot direction.
    heading += angular_speed * dt

    robot.rotation = -heading


    # Convert robot direction into X/Y movement.
    angle = math.radians(
        heading
    )

    movement_x = (
        linear_speed
        * math.cos(angle)
        * dt
    )

    movement_y = (
        linear_speed
        * math.sin(angle)
        * dt
    )


    # --------------------------------------------------
    # TRY X MOVEMENT
    # --------------------------------------------------

    old_x = robot.x

    robot.x += movement_x

    if robot_hits_obstacle():

        robot.x = old_x


    # --------------------------------------------------
    # TRY Y MOVEMENT
    # --------------------------------------------------

    old_y = robot.y

    robot.y += movement_y

    if robot_hits_obstacle():

        robot.y = old_y


    keep_robot_in_window()


# --------------------------------------------------
# DRAW EVERYTHING
# --------------------------------------------------

@window.event
def on_draw():

    window.clear()

    # Draw every obstacle.
    for obstacle in obstacles:
        obstacle.draw()

    # Draw robot last.
    robot.draw()


# --------------------------------------------------
# START SIMULATOR
# --------------------------------------------------

pyglet.clock.schedule_interval(
    update,
    1 / 60
)

pyglet.app.run()