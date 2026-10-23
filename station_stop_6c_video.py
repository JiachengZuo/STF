"""6c adapter for the shared run.py BEV + front-camera video layout."""
from emergency_brake_5d import project, route_point_at
from emergency_brake_5d_video import StandardVideo

VIDEO_PROFILE = 'run_visualizer_bev_camera_512x256_20fps_h264_v1'
VIDEO_METADATA = dict(profile=VIDEO_PROFILE, layout='BEV_left_camera_right',
                      width=512, height=256, fps=20, codec='h264',
                      pixel_format='yuv420p')


def lookahead_xy(route, position):
    projection = project(position, route)
    length = projection['length']
    points = []
    for offset in range(12):
        station = projection['s'] + offset
        # route_point_at can overrun the final segment by floating-point noise.
        point = route[-1] if station >= length - 1e-6 else route_point_at(route, station)
        points.append((point['x'], point['y']))
    return points


class StationStopVideo(StandardVideo):
    def frame(self, rgb, route):
        location = self.viz.ego.get_location()
        self.viz.front_cam_data = rgb
        self.pygame.event.pump()
        self.viz.render(lookahead_xy(route, {'x': location.x, 'y': location.y}))
        frame = self.pygame.surfarray.array3d(self.viz.screen).swapaxes(0, 1)
        if frame.shape != (256, 512, 3):
            raise RuntimeError('shared Visualizer dimensions differ from expected 6c video profile')
        return frame
