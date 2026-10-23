"""2d video using the shared Visualizer and the saved VUT route."""
from cross_right_2d import at, project
from emergency_brake_5d_video import StandardVideo


class CrossRightVideo(StandardVideo):
    def frame(self, rgb, route):
        location = self.viz.ego.get_location()
        p = project(dict(x=location.x, y=location.y), route)
        points = [at(route, min(p['length'], p['s']+i)) for i in range(12)]
        self.viz.front_cam_data = rgb
        self.pygame.event.pump()
        self.viz.render([(p['x'], p['y']) for p in points])
        return self.pygame.surfarray.array3d(self.viz.screen).swapaxes(0, 1)
