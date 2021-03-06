
class Settings():
    """ setting for this game"""

    def __init__(self):

        #
        self.screen_width = 1200
        self.screen_height = 800
        self.bg_color = (230, 230, 230)
        
        # Ship        
        self.ship_limit = 3

        # Bullet       
        self.bullet_width = 300*2
        self.bullet_height = 15
        self.bullet_color = (60, 60, 60)
        self. bullets_allowed = 3

        # Alien
        self.fleet_drop_speed = 10
        self.fleet_direction = 1

        # Level up
        self.speedup_scale = 1.1
        self.initialize_dynamic_settings()

        # Score
        self.alien_points = 50
        self.score_scale = 1.5

    def initialize_dynamic_settings(self):
        """ """
        self.ship_speed_factor = 1.5
        self.bullet_speed_factor = 3
        self.alien_speed_factor = 1

    def increase_speed(self):
        """ """
        self.ship_speed_factor *= self.speedup_scale
        self.bullet_speed_factor *= self.speedup_scale
        self.alien_speed_factor *= self.speedup_scale
        self.alien_points = int(self.score_scale * self.alien_points)
        print(self.alien_points)