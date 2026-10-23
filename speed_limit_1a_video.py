"""The shared run.py BEV + camera layout with frame-matched 1a route overlay."""
from emergency_brake_5d_video import StandardVideo

VIDEO_PROFILE = 'run_visualizer_bev_camera_512x256_20fps_h264_v1'
VIDEO_METADATA = {'profile':VIDEO_PROFILE,'layout':'BEV_left_camera_right',
                  'width':512,'height':256,'fps':20,'codec':'h264','pixel_format':'yuv420p'}


class SpeedLimitVideo(StandardVideo):
    def frame(self, rgb, route):
        location=self.viz.ego.get_location()
        station,_,_=route.project(location.x,location.y)
        points=[route.at(min(route.stations[-1],station+i)) for i in range(12)]
        self.viz.front_cam_data=rgb
        self.pygame.event.pump()
        self.viz.render([(p['x'],p['y']) for p in points])
        frame=self.pygame.surfarray.array3d(self.viz.screen).swapaxes(0,1)
        if frame.shape!=(256,512,3):
            raise RuntimeError('shared Visualizer dimensions differ from expected video profile')
        return frame
