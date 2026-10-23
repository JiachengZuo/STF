"""5e video overlay with endpoint-safe route sampling."""
from curve_emergency_5e import at, project
from emergency_brake_5d_video import StandardVideo


class CurveVideo(StandardVideo):
    def frame(self, rgb, route):
        location = self.viz.ego.get_location()
        projection = project({'x': location.x, 'y': location.y}, route)
        points = [at(route, min(projection['length'], projection['s']+i)) for i in range(12)]
        self.viz.front_cam_data = rgb
        self.pygame.event.pump()
        self.viz.render([(p['x'], p['y']) for p in points])
        return self.pygame.surfarray.array3d(self.viz.screen).swapaxes(0, 1)
