"""Find real gamepads among the Linux joystick devices (/dev/input/js*).

The kernel's joystick interface also exposes touchscreens, touchpads and
some mice: on the lab's Jetson, /dev/input/js0 is the ILITEK touch panel
("ILITEK ILITEK-TP Mouse", udev ID_INPUT_MOUSE).  Its two axes are the
absolute touch position, so reading it as a stick drove the robot whenever
the screen was touched (the reported spin after enabling joystick input).

A device counts as a gamepad when udev tags it ID_INPUT_JOYSTICK and it
is not tagged as a mouse, touchpad, touchscreen or tablet.  Without udev
data, its name must not look like a pointer device and it needs at least
two axes and two buttons.
"""
import array
import fcntl
import glob
import os
import re

_POINTER_TAGS = ('ID_INPUT_MOUSE', 'ID_INPUT_TOUCHPAD', 'ID_INPUT_TOUCHSCREEN',
                 'ID_INPUT_TABLET', 'ID_INPUT_POINTINGSTICK')
_POINTER_NAME = re.compile(r'mouse|touch|tablet|trackpad|stylus|pen\b|-tp\b|\btp\b', re.IGNORECASE)

# <linux/joystick.h>
_JSIOCGAXES = 0x80016a11
_JSIOCGBUTTONS = 0x80016a12


def _jsiocgname(length):
    return 0x80006a13 + (0x10000 * length)


def _udev_properties(path, udev_root='/run/udev/data'):
    """udev E: properties of a character device, or None when unavailable."""
    try:
        rdev = os.stat(path).st_rdev
    except OSError:
        return None
    record = os.path.join(udev_root, 'c%d:%d' % (os.major(rdev), os.minor(rdev)))
    try:
        with open(record) as stream:
            lines = stream.read().splitlines()
    except OSError:
        return None
    properties = {}
    for line in lines:
        if line.startswith('E:') and '=' in line:
            key, value = line[2:].split('=', 1)
            properties[key] = value
    return properties


def _ioctl_identity(path):
    """(name, axes, buttons) from the joystick ioctls; None on failure."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        name = array.array('B', [0] * 128)
        fcntl.ioctl(fd, _jsiocgname(len(name)), name)
        count = array.array('B', [0])
        fcntl.ioctl(fd, _JSIOCGAXES, count)
        axes = count[0]
        fcntl.ioctl(fd, _JSIOCGBUTTONS, count)
        return name.tobytes().split(b'\0', 1)[0].decode(errors='replace'), axes, count[0]
    except OSError:
        return None
    finally:
        os.close(fd)


def classify(name, axes, buttons, properties):
    """(is_gamepad, reason) for one device description."""
    if properties is not None:
        pointer = [tag for tag in _POINTER_TAGS if properties.get(tag) == '1']
        if pointer:
            return False, 'pointer device (%s)' % ', '.join(pointer)
        if properties.get('ID_INPUT_JOYSTICK') == '1':
            return True, 'udev joystick'
        return False, 'not tagged as a joystick by udev'
    if _POINTER_NAME.search(name or ''):
        return False, 'name looks like a pointer device'
    if axes < 2 or buttons < 2:
        return False, 'too few axes/buttons for a gamepad'
    return True, 'joystick interface (no udev data)'


def joystick_devices(pattern='/dev/input/js*', udev_root='/run/udev/data', identify=_ioctl_identity):
    """Every joystick-interface device with its classification."""
    devices = []
    for path in sorted(glob.glob(pattern)):
        identity = identify(path)
        if identity is None:
            devices.append(dict(path=path, name='', axes=0, buttons=0,
                                gamepad=False, reason='cannot be opened'))
            continue
        name, axes, buttons = identity
        gamepad, reason = classify(name, axes, buttons, _udev_properties(path, udev_root))
        devices.append(dict(path=path, name=name, axes=axes, buttons=buttons,
                            gamepad=gamepad, reason=reason))
    return devices


def find_gamepad(**kwargs):
    """The first real gamepad device dict, or None."""
    return next((device for device in joystick_devices(**kwargs) if device['gamepad']), None)
