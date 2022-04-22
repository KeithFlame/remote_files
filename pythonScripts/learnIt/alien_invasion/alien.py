import pygame
from pygame.sprite import Sprite

class Alien(Sprite):
    """ a alien"""
    def __init__(self, ai_settings, screen):
        super().__init__()

        self.screen = screen
        self.ai_settings = ai_settings

        # load alien
        self.image = pygame.image.load('./images/aship.bmp')
        self.rect = self.image.get_rect()

        # position
        self.rect.x = self.rect.width
        self.rect.y = self.rect.height

        # precise position of alien
        self.x = float(self.rect.x)
        self.y = float(self.rect.y)
        self.moving_flag = True

    def blitme(self):
        """ draw"""
        self.screen.blit(self.image, self.rect)
    
    def update(self):
        """ """
        # flag = True
        # if self.rect.right < self.screen.get_rect().right and self.moving_flag == True:
        #     # self.rect.centerx += 1
        #     self.x += self.ai_settings.alien_speed_factor
        # else:
        #     self.moving_flag = False
        # if  self.rect.left > 0 and self.moving_flag == False:
        #     # self.rect.centerx -= 1
        #     self.x -= self.ai_settings.alien_speed_factor
        # else:
        #     self.moving_flag = True
        
        self.x += (self.ai_settings.alien_speed_factor * self.ai_settings. fleet_direction)

        self.rect.centerx = self.x
    
    def check_edges(self):
        """ """
        screen_rect = self.screen.get_rect()
        if self.rect.right >= screen_rect.right or self.rect.left <= 0:
            return True
        else:
            return False

