"""

Frame Viewer

This script displays frames from the OV7670 camera a window
using the Simple DirectMedia Layer (SDL) framework.
The frames are received through the serial port connection from
the Arduino of the CameraBot.
The color space of the incoming frames is expected to be YUV422.
In the yuv422 color space, 2 pixels are encoded in 4 bytes as follows:
<Y1><U><Y2><V>
The first pixel has color <Y0><U><V> in YUV color space
The second pixel has color <Y1><U><V> in YUV color space
See <https://paulbourke.net/dataformats/yuv/>

The start of each frame is preceded by the series of three bytes "VSY" for synchronization purposes.

Install the Python required modules with:
    > python -m pip install pysdl2 numpy pyserial

On Windows, run this script from the directory holding this file,
so that it can find the sdl2.dll file.

On Linux or Mac, install SDL2:
    Ubuntu: sudo apt install libsdl2-dev libsdl2-2.0-0 -y;
    Mac:    brew install SDL2

Do not forget to configure the serial port device below.

"""

import os
import ctypes
import sys
from sys import platform
import serial
import numpy as np
from PIL import Image
import face_recognition_models
import face_recognition
import webbrowser

if platform == "win32":
    os.environ["PYSDL2_DLL_PATH"] = "."
from sdl2 import *
import sdl2.ext

# CONFIGURE ME! (Windows)
# Set the correct COM port used by the USB connection to the XBee dongle
# CONFIGURE ME! (Linux/Mac)
# SERIAL_DEVICE = '/dev/ttyACM0'
SERIAL_DEVICE = '/dev/tty.usbserial-A800f3nS'

# Set the baud rate used by the serial connection
SERIAL_BAUD_RATE = 115200

# CONFIGURE ME! (Linux/Mac)
# SERIAL_DEVICE = '/dev/ttyACM0'

# QVGA image size settings
WIDTH = 240
HEIGHT = 320

global serial_connection

# Preload known celebrity encodings and names
KNOWN_IMAGES = [
    ("https://en.wikipedia.org/wiki/Mark_Zuckerberg", "known_celebrities/mark.jpg"),
    ("https://en.wikipedia.org/wiki/Elon_Musk", "known_celebrities/elon.jpg"),
    ("https://en.wikipedia.org/wiki/Linus_Torvalds", "known_celebrities/linus.jpg"),
    ("https://en.wikipedia.org/wiki/Bill_Gates", "known_celebrities/bill.jpg")
]

known_encodings = []
known_names = []

for name, image_path in KNOWN_IMAGES:
    image = face_recognition.load_image_file(image_path)
    encoding = face_recognition.face_encodings(image)[0]
    known_encodings.append(encoding)
    known_names.append(name)


def yuv2rgb(y, u, v):
    """Convert YUV color space values to to RGB values.
    In the yuv422 color space, 2 pixels are encoded in 4 bytes as follows:
    <Y1><U><Y2><V>
    first pixel <Y0><U><V>
    second pixel <Y1><U><V>
    see <https://paulbourke.net/dataformats/yuv/>
    """

    return (
        np.clip((298 * (y - 16) + 409 * (v - 128) + 128) >> 8, 0, 255),
        np.clip((298 * (y - 16) - 100 * (u - 128) - 208 * (v - 128) + 128) >> 8, 0, 255),
        np.clip((298 * (y - 16) + 516 * (u - 128) + 128) >> 8, 0, 255)
    )


def sync(serial_conn, first=True):
    """ Wait for frame synchronization input """
    # check if we are unexpectedly skipping data
    dropped_bytes = 0

    waiting = True
    while waiting:
        # check for the anticipated sequence
        if serial_conn.read() == b'V':
            if serial_conn.read() == b'S':
                if serial_conn.read() == b'Y':
                    waiting = False
        else:
            # we got something unexpected (unless it is the first frame)
            dropped_bytes = dropped_bytes + 1
    # only on the first frame we expect to drop some data
    if not first:
        if dropped_bytes > 0:
            print(f"Warning: dropped bytes: {dropped_bytes}")


def setupSDL():
    """ Create the SDL window """
    SDL_Init(SDL_INIT_EVERYTHING)
    window = SDL_CreateWindow(b"Frame Viewer",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              WIDTH, HEIGHT, SDL_WINDOW_SHOWN)
    window_surface = SDL_GetWindowSurface(window)
    frame = SDL_CreateRGBSurface(0, WIDTH, HEIGHT, 32, 0x0000ff, 0x00ff00, 0xff0000, 0)
    pixels = sdl2.ext.pixels2d(frame.contents)  # used to access pixels from passed source directly
    SDL_BlitSurface(frame, None, window_surface, None)  # copy the frame to the window surface
    SDL_UpdateWindowSurface(window)  # copy the window surface to the screen
    event = SDL_Event()

    return [pixels, frame, window_surface, window, event]


def updateSDLWindow(frame, window_surface, window, event):
    """ Update the window with new image data """

    # copy frame to the window surface
    SDL_BlitSurface(frame, None, window_surface, None)
    # copy the window surface to the screen
    SDL_UpdateWindowSurface(window)
    # check for any quit events
    while SDL_PollEvent(ctypes.byref(event)) != 0:
        if event.type == SDL_QUIT:
            return False
    return True


def cleanupSDL(frame, window):
    """ Close the window """
    SDL_FreeSurface(frame)
    SDL_DestroyWindow(window)
    SDL_Quit()


def save_frame_as_image(pixels, frame_number):
    """Save the current frame as an image file."""
    # Convert the pixels array to a numpy array of shape (HEIGHT, WIDTH, 3) for RGB
    array = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    for x in range(WIDTH):
        for y in range(HEIGHT):
            pixel_data = pixels[x][y]
            r = (pixel_data >> 16) & 0xFF
            g = (pixel_data >> 8) & 0xFF
            b = pixel_data & 0xFF
            array[y, x] = [r, g, b]  # NOTICE: Y (row), X (column)

    # Save the array as an image using Pillow
    img = Image.fromarray(array)
    output_dir = "captured_images"
    os.makedirs(output_dir, exist_ok=True)  # Ensure the directory exists
    filename = f"{output_dir}/frame_{frame_number}.png"
    img.save(filename)
    print(f"Saved frame {frame_number} to {filename}")

    # Perform face recognition
    target_image = face_recognition.load_image_file(filename)
    target_encoding = face_recognition.face_encodings(target_image)

    if target_encoding:
        distances = face_recognition.face_distance(known_encodings, target_encoding[0])
        best_match_index = distances.argmin()
        print(f"Recognized: {known_names[best_match_index]} with distance {distances[best_match_index]}")
        webbrowser.open(known_names[best_match_index])
    else:
        print("No face detected in the frame.")


def stage1():  # receive text
    while True:
        try:
            serial_input = serial_connection.read().decode()
        except UnicodeDecodeError:
            continue
        if serial_input == "Q":
            break
        print(serial_input, end='')


def stage2():  # receive images
    # Setup the SDL window
    (pixels, frame, window_surface, window, event) = setupSDL()

    # Start receiving image frames.

    # wait until the sync key is received via the serial port
    sync(serial_connection, first=True)

    frame_number = 0
    continue_running = True
    while continue_running:
        # for every frame column
        for x in range(0, WIDTH, 1):
            # for every frame row y from HEIGHT-1 down to 0 with 2 pixels per step
            for y in range(HEIGHT - 1, 0, -2):
                # read 4 bytes for 2 pixels from the serial port
                buf = serial_connection.read(4)
                # convert pixel 1 YUV to RGB
                (r, g, b) = yuv2rgb(buf[0], buf[1], buf[3])
                # set the pixel data in the SDL window
                pixels[x][y] = (r << 16) | (g << 8) | (b << 0)
                # convert pixel 2 YUV to RGB
                (r, g, b) = yuv2rgb(buf[2], buf[1], buf[3])
                # set the pixel data in the SDL window
                pixels[x][y - 1] = (r << 16) | (g << 8) | (b << 0)

            # update the window (once for every column of new pixels)
            continue_running = continue_running and updateSDLWindow(frame, window_surface, window, event)

            if not continue_running:
                break
        if not continue_running:
            break

        save_frame_as_image(pixels, frame_number)
        frame_number += 1

        # wait until the sync key is received for the next frame via the serial port
        sync(serial_connection, first=False)

    # close the window
    cleanupSDL(frame, window)
    return 0


def main():
    global serial_connection

    # Open the serial connection
    try:
        serial_connection = serial.Serial(SERIAL_DEVICE, SERIAL_BAUD_RATE)
    except serial.SerialException as e:
        print(f"Exception: {e}")
        print("Serial connection not established. \nCheck port number and baudrate.")
        sys.exit(-1)

    stage1()
    stage2()

    # close the serial connection
    serial_connection.close()


if __name__ == "__main__":
    sys.exit(main())
