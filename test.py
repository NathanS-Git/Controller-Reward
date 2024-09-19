import pygame

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks: {joystick_count}")

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick name: {joystick.get_name()}")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")
            elif event.type == pygame.JOYAXISMOTION:
                print(f"Axis {event.axis} moved to {event.value}")
else:
    print("No joysticks found")
