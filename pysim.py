#!/usr/bin/env python

import pyglet

from src.windows.startwindow import StartWindow
from src.windows.Ben_simulator import Simulator


if __name__ == "__main__":

    #Tkinter menu starts up, is paused when simulation is running, and resumes when simulation ends

    start_window = StartWindow()

    try:
        while True:

            # Refresh the XML world list 

            start_window.refresh_world_filelist()

            selected_file, selected_robot = start_window.start()
            # Reopen tkinter window with selected map/robot
            if (
                not selected_file
                or selected_file == "None"
                or not selected_robot
                or selected_robot == "None"
            ):
                break

            print("starting simulator")
            print(selected_file, selected_robot)

            # Start simulator
            simulator = Simulator(
                selected_file,
                selected_robot,
                start_window
            )

            #30Hz clock
            pyglet.clock.schedule_interval(
                simulator.update,
                1 / 30
            )

    
            pyglet.app.run()

            pyglet.clock.unschedule(
                simulator.update
            )

            simulator = None

            # loop repeats and returns to the Tkinter menu

    except KeyboardInterrupt:
        print("Goodbye!")