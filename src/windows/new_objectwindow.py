     
# world editor toolbar
     
#
# this file creates the separate Objects window used when EDIT is enabled
# in Ben_simulator.py
#
# overall flow:
#
#   Ben_simulator.py
#       -> creates ObjectWindow
#       -> passes it a reference to the main Simulator
#
#   ObjectWindow
#       -> loads the available editor images/resources
#       -> creates SelectSprite objects for each toolbar item
#       -> handles selecting and dragging tools
#       -> sends preview/place requests back to Simulator
#
#   selectablesprite.py
#       -> represents each selectable item shown in this toolbar
#
#
# toolbar contains:
#
#   object
#       static obstacle that can be placed in the world
#
#   line_map
#       replaces the current line sensor map
#
#   background
#       replaces the current world background
#
#   delete
#       removes an existing world object
#
#   light
#       places the light source used by the light sensors
#
#
# drag behaviour:
#
#   left click + hold toolbar item
#       -> selects the tool
#       -> converts the toolbar mouse position into simulator coordinates
#       -> asks Simulator to display a transparent preview
#
#   release inside simulator
#       -> places/applies the selected item
#
# clicking an item also keeps the original selected-tool behaviour so
# right-click placement in the simulator can still be used
#
#
# important OpenGL note:
#
# this toolbar and the simulator are separate pyglet windows, so they
# have separate OpenGL contexts
#
# calls into Simulator may switch to the simulator context to create or
# remove sprites. this file therefore switches back to its own context
# afterwards before drawing toolbar sprites
#
# shadow_window is disabled because context sharing was one of the
# problems encountered with the original pyglet code
#
#
# main variables:
#
#   SPACING
#       distance between toolbar item positions
#
#   PADDING
#       space around toolbar contents
#
#   THUMB_SIZE
#       maximum display size used for large toolbar thumbnails
#
#   main_batch
#       pyglet batch containing the toolbar sprites
#
#   sprites
#       list of SelectSprite toolbar items
#
#   drag_sprite
#       toolbar item currently being dragged
#
#   close_me
#       tells the main Simulator that this toolbar was closed using X
#
#   repo_folder
#       path to the main Robot-Workshop folder/resources
#
#   simulator
#       reference back to the main Simulator window
     

from pathlib import Path

import pyglet

# disables shadow window - avoids the context sharing issue seen with the old code
pyglet.options["shadow_window"] = False

from pyglet.window import key, mouse

from src.sprites.selectablesprite import SelectSprite


# toolbar layout
SPACING = 50
PADDING = 8
THUMB_SIZE = 48


# moves image anchor to midpoint
def centre_image(image):
    image.anchor_x = image.width // 2
    image.anchor_y = image.height // 2


# separate window used to hold the world editing tools
class ObjectWindow(pyglet.window.Window):

    def __init__(self, width, height, repo_folder, simulator):

        # create toolbar window
        super().__init__(
            width=width,
            height=height,
            caption="Objects",
            visible=False
        )

        # location of main Robot-Workshop folder
        self.repo_folder = Path(repo_folder)

        # reference to the main simulator window
        self.simulator = simulator

        # sprites are kept in one batch so the toolbar can be drawn together
        self.main_batch = pyglet.graphics.Batch()
        self.sprites = []

        # object currently being dragged from the toolbar
        self.drag_sprite = None

        # simulator checks this if toolbar is closed using its own X button
        self.close_me = False

        self.load_images()
        self.load_sprites()


    def load_images(self):

        # make sure images are created for the editor OpenGL context
        self.switch_to()

        # original static object sprite sheet
        sheet = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "static_objects"
                / "boxesv2.png"
            )
        )

        # split sheet into the 9 original objects
        self.object_images = list(
            pyglet.image.ImageGrid(sheet, 1, 9)
        )

        for image in self.object_images:
            centre_image(image)

        # original line maps (map0 - map9)
        self.line_images = []

        for i in range(10):
            image = pyglet.image.load(
                str(
                    self.repo_folder
                    / "resources"
                    / "line_maps"
                    / f"map{i}.png"
                )
            )

            centre_image(image)
            self.line_images.append(image)

        # original background images
        self.background_images = []

        for i in range(4):
            image = pyglet.image.load(
                str(
                    self.repo_folder
                    / "resources"
                    / "backgrounds"
                    / f"bg{i}.png"
                )
            )

            centre_image(image)
            self.background_images.append(image)

        # delete tool
        self.erase_image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "robot"
                / "erase.png"
            )
        )
        centre_image(self.erase_image)

        # light source tool
        self.light_image = pyglet.image.load(
            str(
                self.repo_folder
                / "resources"
                / "static_objects"
                / "light.png"
            )
        )
        centre_image(self.light_image)


    # finds the position for the next toolbar item
    # fills each column from top to bottom before moving right
    def next_position(self, x, y, width, height):

        y -= height + PADDING

        if y < SPACING / 2:
            x += width + PADDING
            y = self.height - SPACING

        return x, y


    def load_sprites(self):

        # sprites belong to the editor window
        self.switch_to()

        x = SPACING
        y = self.height - SPACING

        # static objects
        for i, image in enumerate(self.object_images):

            sprite = SelectSprite(
                "object",
                i,
                image,
                x,
                y,
                self.main_batch
            )

            self.sprites.append(sprite)

            x, y = self.next_position(
                x,
                y,
                image.width,
                image.height
            )

        # line maps
        for i, image in enumerate(self.line_images):

            # line maps are large, so only show a thumbnail here
            scale = THUMB_SIZE / max(image.width, image.height)

            sprite = SelectSprite(
                "line_map",
                i,
                image,
                x,
                y,
                self.main_batch,
                scale
            )

            self.sprites.append(sprite)

            x, y = self.next_position(
                x,
                y,
                THUMB_SIZE,
                THUMB_SIZE
            )

        # backgrounds
        for i, image in enumerate(self.background_images):

            scale = THUMB_SIZE / max(image.width, image.height)

            sprite = SelectSprite(
                "background",
                i,
                image,
                x,
                y,
                self.main_batch,
                scale
            )

            self.sprites.append(sprite)

            x, y = self.next_position(
                x,
                y,
                THUMB_SIZE,
                THUMB_SIZE
            )

        # delete tool
        scale = min(
            1,
            THUMB_SIZE / max(
                self.erase_image.width,
                self.erase_image.height
            )
        )

        sprite = SelectSprite(
            "delete",
            -1,
            self.erase_image,
            x,
            y,
            self.main_batch,
            scale
        )

        self.sprites.append(sprite)

        x, y = self.next_position(
            x,
            y,
            THUMB_SIZE,
            THUMB_SIZE
        )

        # light source
        scale = min(
            1,
            THUMB_SIZE / max(
                self.light_image.width,
                self.light_image.height
            )
        )

        sprite = SelectSprite(
            "light",
            -1,
            self.light_image,
            x,
            y,
            self.main_batch,
            scale
        )

        self.sprites.append(sprite)


    # returns the toolbar object underneath the mouse
    def get_sprite_at(self, x, y):

        for sprite in self.sprites:

            if (
                sprite.x - sprite.width / 2 <= x <= sprite.x + sprite.width / 2
                and
                sprite.y - sprite.height / 2 <= y <= sprite.y + sprite.height / 2
            ):
                return sprite

        return None


    # marks the clicked toolbar item as selected
    def select_sprite(self, selected):

        for sprite in self.sprites:
            sprite.selected = sprite is selected

            if sprite.selected:
                sprite.opacity = 255
            else:
                sprite.opacity = 120


    # still used by the original select then right-click method
    def get_selected_sprite_name(self):

        for sprite in self.sprites:
            if sprite.selected:
                return sprite.object_type, sprite.idx

        return "none", -1


    # converts a mouse position in the toolbar window to the same
    # position in the main simulator window
    def simulator_coordinates(self, x, y):

        tool_x, tool_y = self.get_location()
        sim_x, sim_y = self.simulator.get_location()

        # get_location measures from the top of the desktop,
        # whereas pyglet mouse Y is measured from the bottom of the window
        screen_x = tool_x + x
        screen_y = tool_y + self.height - y

        sim_mouse_x = screen_x - sim_x
        sim_mouse_y = self.simulator.height - (screen_y - sim_y)

        inside = (
            0 <= sim_mouse_x <= self.simulator.width
            and
            0 <= sim_mouse_y <= self.simulator.height
        )

        return sim_mouse_x, sim_mouse_y, inside


    def on_mouse_press(self, x, y, button, modifiers):

        if button != mouse.LEFT:
            return

        # find which toolbar item has been clicked
        sprite = self.get_sprite_at(x, y)

        if sprite is None:
            self.drag_sprite = None
            return

        # clicking also selects it for the original right-click method
        self.select_sprite(sprite)
        self.drag_sprite = sprite


    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):

        if self.drag_sprite is None or not buttons & mouse.LEFT:
            return

        # convert toolbar mouse position into simulator coordinates
        x, y, inside = self.simulator_coordinates(x, y)

        # show a transparent copy in the simulator while dragging
        self.simulator.preview_tool(
            self.drag_sprite.object_type,
            self.drag_sprite.idx,
            x,
            y,
            inside
        )

        # preview_tool switches to simulator OpenGL context
        # switch back before the editor draws again
        self.switch_to()


    def on_mouse_release(self, x, y, button, modifiers):

        if button != mouse.LEFT or self.drag_sprite is None:
            return

        sprite = self.drag_sprite
        x, y, inside = self.simulator_coordinates(x, y)

        # remove the temporary transparent sprite
        self.simulator.clear_preview()

        # add real object if it was released over the simulator
        if inside:
            self.simulator.add_tool(
                sprite.object_type,
                sprite.idx,
                x,
                y
            )

        self.drag_sprite = None

        # simulator functions above change active OpenGL context
        self.switch_to()


    def on_key_press(self, symbol, modifiers):

        # E closes the editor
        if symbol == key.E:
            self.close()


    def on_draw(self):

        # grey makes transparent line-map thumbnails easier to see
        pyglet.gl.glClearColor(0.65, 0.65, 0.65, 1)

        self.clear()
        self.main_batch.draw()


    def on_close(self):

        # tells simulator that edit mode should also be closed
        self.close_me = True
        self.drag_sprite = None

        # if whole world is already closing the simulator handles cleanup
        if not self.simulator.world_closing:
            try:
                self.simulator.clear_preview()
                self.switch_to()
            except Exception:
                pass

        return super().on_close()
