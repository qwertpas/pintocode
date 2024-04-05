import pygame
import sys

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
GREEN = (0, 255, 0)

# Initialize Pygame
pygame.init()

# Set the width and height of the screen (width, height)
size = (800, 600)
screen = pygame.display.set_mode(size)

# Set the window title
pygame.display.set_caption("Pygame Slider Example")

# Loop until the user clicks the close button
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Slider properties
slider_width = 200
slider_height = 20
slider_x = 100
slider_y = 300
slider_color = GRAY
slider_handle_color = GREEN

# Slider value
slider_value = 0.5

def draw_slider(value):
    pygame.draw.rect(screen, slider_color, [slider_x, slider_y, slider_width, slider_height])
    handle_x = slider_x + int(value * slider_width)
    pygame.draw.circle(screen, slider_handle_color, (handle_x, slider_y + slider_height // 2), 10)

# Main loop
while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if slider_x <= event.pos[0] <= slider_x + slider_width and \
               slider_y <= event.pos[1] <= slider_y + slider_height:
                # Calculate new slider value based on mouse position
                mouse_x = event.pos[0]
                relative_x = mouse_x - slider_x
                slider_value = max(0, min(1, relative_x / slider_width))
        elif event.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]:  # If left mouse button is pressed
                # Calculate new slider value based on mouse position
                mouse_x = event.pos[0]
                relative_x = mouse_x - slider_x
                slider_value = max(0, min(1, relative_x / slider_width))
    
    # --- Game logic should go here
    print(slider_value)
    
    # --- Drawing code should go here
    screen.fill(WHITE)
    draw_slider(slider_value)

    # --- Update the screen
    pygame.display.flip()

    # --- Limit frames per second
    clock.tick(60)

# Close the window and quit.
pygame.quit()
sys.exit()
