"""Log every button index on the pad so the real L1/R1 indices can be identified."""
import os
import time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise SystemExit("no pad visible")

pad = pygame.joystick.Joystick(0)
pad.init()
count = pad.get_numbuttons()

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "buttons.csv")
started = time.time()
with open(path, "w") as out:
    out.write("t," + ",".join("b%d" % i for i in range(count)) + "\n")
    while time.time() - started < 150:
        pygame.event.pump()
        values = [str(pad.get_button(i)) for i in range(count)]
        out.write("%.2f,%s\n" % (time.time() - started, ",".join(values)))
        out.flush()
        time.sleep(0.05)
