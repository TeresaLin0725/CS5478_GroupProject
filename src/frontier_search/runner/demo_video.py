"""Stream real simulation frames to H.264 without retaining them in memory."""
from pathlib import Path


class DemoVideo:
    def __init__(self, path, fps=15):
        import imageio_ffmpeg
        self.path, self.fps = Path(path), fps
        if fps <= 0 or 30 % fps:
            raise ValueError('video fps must divide the 30 Hz control rate')
        self.stride = int(round(30 / fps))
        self.frames, self.last_step = [], -1
        self.writer = imageio_ffmpeg.write_frames(str(self.path), (1440, 900), fps=fps,
            codec='libx264', pix_fmt_out='yuv420p', macro_block_size=2,
            output_params=['-preset', 'veryfast', '-crf', '18', '-movflags', '+faststart'],
            ffmpeg_log_level='error')
        self.writer.send(None)

    def due(self, step):
        return step != self.last_step and step % self.stride == 0

    def capture(self, picture, world, caption):
        if world.steps == self.last_step:
            return
        self.writer.send(picture.tobytes())
        pose = world.pose()
        self.frames.append({'time_s': round(world.sim_time, 6),
            'distance_m': round(world.distance, 3), 'contacts': world.collision_steps,
            'pose': {'x_m': pose.x_m, 'y_m': pose.y_m, 'yaw_rad': pose.yaw_rad}, 'caption': caption})
        self.last_step = world.steps

    def close(self):
        if self.writer is not None:
            self.writer.close()
            self.writer = None

    def metadata(self):
        gaps = [b['time_s']-a['time_s'] for a,b in zip(self.frames,self.frames[1:])]
        return {'file': self.path.name, 'fps': self.fps, 'frames': self.frames,
                'max_gap_s': round(max(gaps, default=0), 6), 'source': 'continuous real simulator captures'}
