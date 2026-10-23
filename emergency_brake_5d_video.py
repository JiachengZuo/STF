"""5d adapter for the shared run visualization, with frame-matched RGB input."""
import os


class StandardVideo:
    def __init__(self, world, ego, headless=False):
        if headless:
            os.environ['SDL_VIDEODRIVER'] = 'dummy'
        import pygame
        from render import Visualizer
        pygame.display.init()

        class SynchronizedVisualizer(Visualizer):
            def _spawn_rgb_camera(self):
                # BrakeScene owns the camera and waits for the matching frame.
                return None

        self.pygame = pygame
        self.viz = SynchronizedVisualizer(world, ego)

    def frame(self, rgb, route):
        from emergency_brake_5d import project, route_point_at
        location = self.viz.ego.get_location()
        projection = project({'x': location.x, 'y': location.y}, route)
        points = [route_point_at(route, min(projection['length'], projection['s']+i))
                  for i in range(12)]
        self.viz.front_cam_data = rgb
        self.pygame.event.pump()
        self.viz.render([(p['x'], p['y']) for p in points])
        return self.pygame.surfarray.array3d(self.viz.screen).swapaxes(0, 1)

    def close(self):
        self.viz = None
        self.pygame.display.quit()
