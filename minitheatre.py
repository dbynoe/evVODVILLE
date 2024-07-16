# Mini theatre script for running a one button video player
#    Copyright (C) 2024 David Bynoe
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import RPi.GPIO as GPIO
import os
import sys
from subprocess import Popen, PIPE, call
import time
from threading import Lock
import signal
import argparse
import pigpio
pi = pigpio.pi() # connect to local Pi
lastpress = time.time()


# House lights - 12v mosfet 
lpin = 18

#Brightness range
lmin = 3 
lmax = 255

#stuff related to the file count
vidcount = 0 #Number of videos available in the usb home directory  
curvideo = 0 #Which video number are we playing at the moment 

# I set things up so I could use a pin to toggle the power button on my display 
# theory is that it saves power/ makes the display last longer
# Its not ideal, there is no feedback so things can get unsynchronized 
state = 0 #current power state of the display 
screenpin  = 17 #pin to toggle the display on/off
sleepwait = 0.1 #Time delay for how long to hold a button press to toggle the display. 


class _GpioParser(argparse.Action):
    """ Parse a GPIO spec string (see argparse setup later in this file) """
    def __call__(self, parser, namespace, values, option_string=None):
        gpio_dict = {}
        pin_pairs = values.split(',')
        for pair in pin_pairs:
            pair_split = pair.split(':')

            if 0 == len(pair_split) > 2:
                raise ValueError('Invalid GPIO pin format')

            try:
                in_pin = int(pair_split[0])
            except ValueError:
                raise ValueError('GPIO input pin must be numeric integer')

            try:
                out_pin = int(pair_split[1])
            except ValueError:
                raise ValueError('GPIO output pin must be numeric integer')
            except IndexError:
                out_pin = None

            if in_pin in gpio_dict:
                raise ValueError('Duplicate GPIO input pin: {}'.format(in_pin))

            gpio_dict[in_pin] = out_pin

        setattr(namespace, self.dest, gpio_dict)


class VidLooper(object):
    _GPIO_BOUNCE_TIME = 200
    _VIDEO_EXTS = ('.mp4', '.m4v', '.mov', '.avi', '.mkv')
    _GPIO_PIN_DEFAULT = {
        26: 21,
        19: 20,
        13: 16,
        6: 12
    }

    # Use this lock to avoid multiple button presses updating the player
    # state simultaneously
    _mutex = Lock()

    # The currently playing video filename
    _active_vid = None


    # The process of the active video player
    _p = None

    def __init__(self, audio='hdmi', autostart=True, restart_on_press=False,
                 video_dir=os.getcwd(), videos=None, gpio_pins=None, loop=True,
                 no_osd=False, shutdown_pin=None, splash=None, debug=False):
        global vidcount
        # Use default GPIO pins, if needed
        if gpio_pins is None:
            gpio_pins = self._GPIO_PIN_DEFAULT.copy()
        self.gpio_pins = gpio_pins

        # Add shutdown pin
        self.shutdown_pin = shutdown_pin

        # Assemble the list of videos to play, if needed
        if videos:
            self.videos = videos
            for video in videos:
                if not os.path.exists(video):
                    raise FileNotFoundError('Video "{}" not found'.format(video))
        else:
            self.videos = [os.path.join(video_dir, f)
                           for f in sorted(os.listdir(video_dir))
                           if os.path.splitext(f)[1] in self._VIDEO_EXTS]
            if not self.videos:
                raise Exception('No videos found in "{}". Please specify a different '
                                'directory or filename(s).'.format(video_dir))
        vidcount = len(self.videos)
        self.debug = debug
        assert audio in ('hdmi', 'local', 'both'), "Invalid audio choice"
        self.audio = audio
        self.autostart = autostart
        self.restart_on_press = restart_on_press
        self.loop = loop
        self.no_osd = no_osd
        self.splash = splash
        self._splashproc = None

    def _kill_process(self):
        """ Kill a video player process. SIGINT seems to work best. """
        if self._p is not None:
            os.killpg(os.getpgid(self._p.pid), signal.SIGINT)
            self._p = None

    def switch_vid(self, pin):
        """ Switch to the video corresponding to the shorted pin """
        global state
        global curvideo
        global vidcount
        lightsdown = 0
        global lastpress
        # only allow button presses every 2 seconds to avoid multiple videos because toddlers
        if time.time() > lastpress  + 2:
            lastpress = time.time()
            # Use a mutex lock to avoid race condition when
            with self._mutex:
            filename = self.videos[curvideo]
            curvideo = curvideo + 1
            if curvideo >= vidcount:
                curvideo = 0
                if filename != self._active_vid or self.restart_on_press:
                    # Kill any previous video player process
                    self._kill_process()
                GPIO.output (screenpin, GPIO.HIGH) #toggle the power button
                time.sleep(sleepwait)
                #wait a beat like you got a fat finger
                GPIO.output (screenpin, GPIO.LOW)
                lightsdown = 1 #we are a real theater now, lets dim the lights
                state = 1 # screen is on


                    # Start a new video player process, capture STDOUT to keep the
                    # screen clear. Set a session ID (os.setsid) to allow us to kill
                    # the whole video player process tree.
                    cmd = ['omxplayer','--win', '120,25,590,530', '--orientation', '180', '-b', '--aspect-mode', 'stretch',  '-o', self.audio]
                    if self.loop:
                            cmd += ['--loop']
                    if self.no_osd:
                            cmd += ['--no-osd']         
                    self._p = Popen(cmd + [filename],
                                stdout=None if self.debug else PIPE,
                                preexec_fn=os.setsid)
                    self._active_vid = filename
                    if lightsdown:
                        for p in range(lmax,lmin,-1):
                                pi.set_PWM_dutycycle(lpin, p)
                                time.sleep(0.013)
                        lightsdown  = 0
                        #Spread the dimming over 3, seconds to give the screen time to wake up
    @property
    def in_pins(self):
        """ Create a tuple of input pins, for easy access """
        return tuple(self.gpio_pins.keys())

    def start(self):
        global pi
        global state
        if not state:
            for p in range(lmin, lmax):
                    pi.set_PWM_dutycycle(lpin, p)
                    time.sleep(0.013)

        if not self.debug:
            # Clear the screen
            os.system('clear')
            # Disable the (blinking) cursor
            os.system('tput civis')

        # Set up GPIO
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(screenpin, GPIO.OUT)
        GPIO.output(screenpin, GPIO.HIGH)
        time.sleep(sleepwait) #wait a beat like you got a fat finger
        GPIO.output (screenpin, GPIO.LOW)

        for in_pin, out_pin in self.gpio_pins.items():
            GPIO.setup(in_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


        # Set up the shutdown pin
        if self.shutdown_pin:
            GPIO.setup(self.shutdown_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.shutdown_pin,
                                  GPIO.FALLING,
                                  callback=lambda _: call(['shutdown', '-h', 'now'], shell=False),
                                  bouncetime=self._GPIO_BOUNCE_TIME)

        if self.autostart:
            if self.splash is not None:
                self._splashproc = Popen(['fbi', '--noverbose', '-a',
                                          self.splash])
            else:
                # Start playing first video
                self.switch_vid(self.in_pins[0])

        # Enable event detection on each input pin
        for pin in self.in_pins:
            GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.switch_vid,
                                  bouncetime=self._GPIO_BOUNCE_TIME)

        # Loop forever
        try:
            while True:
                time.sleep(0.5)
                if not self.loop:
                    pid = -1
                    if self._p:
                        pid = self._p.pid
                        self._p.communicate()
                    if self._p:
                        if self._p.pid == pid:
                            # Reset LEDs
                            for out_pin in self.gpio_pins.values():
                                if out_pin is not None:
                                    GPIO.output(out_pin, GPIO.LOW)
                            self._active_vid = None
                            self._p = None
                #if pi.get_PWM_dutycycle(lpin) != lmax:
                if state == 1:
                    GPIO.output (screenpin, GPIO.HIGH)
                        time.sleep(sleepwait)
                        #wait a beat like you got a fat finger
                        GPIO.output (screenpin, GPIO.LOW)
                state=0

                for p in range(lmin, lmax, 1):
                        pi.set_PWM_dutycycle(lpin, p)
                        time.sleep(0.013)

    except KeyboardInterrupt:
        pass
        finally:
            self.__del__()

    def __del__(self):
        if not self.debug:
            # Reset the terminal cursor to normal
            os.system('tput cnorm')

        # Cleanup the GPIO pins (reset them)
        GPIO.cleanup()

        # Kill any active video process
        self._kill_process()

        # Kill any active splash screen
        if self._splashproc:
            os.killpg(os.getpgid(self._splashproc.pid), signal.SIGKILL)


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""Raspberry Pi video player controlled by GPIO pins

This program is designed to power a looping video display, where the active
video can be changed by pressing a button (i.e. by shorting a GPIO pin).
The active video can optionally be indicated by an LED (one output for each
input pin; works well with switches with built-in LEDs, but separate LEDs work
too).

This video player uses omxplayer, a hardware-accelerated video player for the
Raspberry Pi, which must be installed separately.
"""
    )
    parser.add_argument('--audio', default='hdmi',
                        choices=('hdmi', 'local', 'both'),
                        help='Output audio over HDMI, local (headphone jack),'
                             'or both')
    parser.add_argument('--no-autostart', action='store_false',
                        dest='autostart', default=True,
                        help='Don\'t start playing a video on startup')
    parser.add_argument('--no-loop', action='store_false', default=True,
                        dest='loop', help='Don\'t loop the active video')
    parser.add_argument(
        '--restart-on-press', action='store_true', default=False,
        help='If True, restart the current video if the button for the active '
             'video is pressed. If False, pressing the button for the active '
             'video will be ignored.')
    vidmode = parser.add_mutually_exclusive_group()
    vidmode.add_argument(
        '--video-dir', default=os.getcwd(),
        help='Directory containing video files. Use this or specify videos one '
             'at a time at the end of the command.')
    vidmode.add_argument('videos', action="store", nargs='*', default=(),
                         help='List of video paths (local, rtsp:// or rtmp://)')
    parser.add_argument('--gpio-pins', default=VidLooper._GPIO_PIN_DEFAULT,
                        action=_GpioParser,
                        help='List of GPIO pins. Either INPUT:OUTPUT pairs, or '
                             'just INPUT pins (no output), separated by '
                             'commas.')
    parser.add_argument('--debug', action='store_true', default=False,
                        help='Debug mode (don\'t clear screen or suppress '
                             'terminal output)')
    parser.add_argument('--countdown', type=int, default=0,
                        help='Add a countdown before start (time in seconds)')
    parser.add_argument('--splash', type=str, default=None,
                        help='Splash screen image to show when no video is '
                             'playing')
    parser.add_argument('--no-osd', action='store_true', default=False,
                        help='Don\'t show on-screen display when changing '
                             'videos')
    parser.add_argument('--shutdown-pin', type=int, default=None,
                        help='GPIO pin to trigger system shutdown (default None)')

    # Invoke the videoplayer
    args = parser.parse_args()

    # Apply any countdown
    countdown = args.countdown

    while countdown > 0:
        sys.stdout.write(
            '\rrpi-vidlooper starting in {} seconds '
            '(Ctrl-C to abort)...'.format(countdown))
        sys.stdout.flush()
        time.sleep(1)
        countdown -= 1

    del args.countdown

    VidLooper(**vars(args)).start()


if __name__ == '__main__':
    main()
