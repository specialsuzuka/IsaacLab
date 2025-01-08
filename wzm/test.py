import pygame

pygame.init()
screen = pygame.display.set_mode((400, 300))

# Example control loop for updating DOF
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                print("UP")
            elif event.key == pygame.K_DOWN:
                print("DOWN")