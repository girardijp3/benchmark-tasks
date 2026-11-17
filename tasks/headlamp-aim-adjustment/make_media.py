"""Render the MuJoCo demo of headlamp-aim-adjustment into the media package.

Top view from the station camera, with a 3.00 m beam-screen inset rendered from a free camera in front of the
screen.  Writes setup.png, the composite frames (for demo.mp4), and the top-view frames (for demo_top.gif).
Run:  cd local_sim && PYTHONPATH=. .venv/bin/python ../benchmark-submission/tasks/headlamp-aim-adjustment/make_media.py
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from mjkit.run import run
from mjkit.usd_import import COL_GROUP

HERE = Path(__file__).resolve().parent
MEDIA = HERE.parents[2] / "benchmark-media-headlamp-aim-adjustment"
FRAMES = MEDIA / "frames"
TOP = MEDIA / "top_frames"
TASK = HERE / "headlamp_aim_adjustment.py"

W, H = 640, 360
INSET_W, INSET_H = 280, 158
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def font(size, bold=True):
    try:
        return ImageFont.truetype(FONT if bold else FONT_R, size)
    except OSError:
        return ImageFont.load_default()


class Media:
    def __init__(self):
        self.screen_renderer = None
        self.cam = None
        self.n = 0
        self.last = None
        self.first = None
        FRAMES.mkdir(parents=True, exist_ok=True)
        TOP.mkdir(parents=True, exist_ok=True)
        for p in FRAMES.glob("*.png"):
            p.unlink()
        for p in TOP.glob("*.png"):
            p.unlink()

    def frame(self, sim, step, states, info):
        top = sim.render("top_cam", width=W, height=H)
        if self.screen_renderer is None:
            self.screen_renderer = mujoco.Renderer(sim.model, 360, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.lookat = np.array([3.508, -0.130, 0.860])
            self.cam.distance = 1.30
            self.cam.azimuth = 0.0
            self.cam.elevation = 4.0
        self.screen_renderer.update_scene(sim.data, camera=self.cam, scene_option=sim._vopt)
        screen = cv2.resize(self.screen_renderer.render(), (INSET_W, INSET_H))
        img = Image.fromarray(top)
        d = ImageDraw.Draw(img)
        # header
        d.rectangle([0, 0, W, 24], fill=(18, 22, 28))
        d.text((8, 4), "top view: Hella 1058 H4 aim bench", font=font(12), fill=(245, 245, 240))
        # screen inset
        x0, y0 = W - INSET_W - 10, 30
        img.paste(Image.fromarray(screen), (x0, y0))
        d.rectangle([x0 - 2, y0 - 2, x0 + INSET_W + 1, y0 + INSET_H + 1], outline=(245, 245, 240), width=2)
        d.rectangle([x0 - 2, y0 - 22, x0 + INSET_W + 1, y0 - 2], fill=(18, 22, 28))
        d.text((x0 + 4, y0 - 19), "beam screen, 3.00 m", font=font(11), fill=(245, 245, 240))
        # key turns and cutoff
        v = float(states.get("v_turn", 0.0)) * 180 / np.pi
        h = float(states.get("h_turn", 0.0)) * 180 / np.pi
        ve = float(states.get("v_err", 0.0))
        he = float(states.get("h_err", 0.0))
        t = float(states.get("episode_time", 0.0))
        d.rectangle([0, H - 46, 330, H], fill=(18, 22, 28))
        d.text((8, H - 42), f"V screw {v:+6.1f} deg   H screw {h:+7.1f} deg", font=font(12), fill=(240, 240, 235))
        d.text((8, H - 24), f"cutoff  {ve:+5.1f} mm vertical, {he:+5.1f} mm horizontal", font=font(12, False),
               fill=(230, 230, 225))
        d.text((W - 92, H - 22), f"{t:5.1f} s", font=font(12), fill=(230, 230, 225))
        score = float(info.get("score", 0.0))
        # ladder (top-left; the arms stay below it)
        for k, (lab, on) in enumerate((("V engaged", score >= 0.25), ("V in band", score >= 0.5),
                                        ("H in band", score >= 0.75), ("hold 1 s", score >= 1.0))):
            yy = 32 + k * 20
            col = (60, 190, 90) if on else (80, 84, 90)
            d.rectangle([8, yy, 20, yy + 14], fill=col)
            d.text((26, yy - 1), lab, font=font(12, False), fill=(240, 240, 235))
        if score >= 1.0:
            d.rectangle([250, 268, 475, 296], fill=(20, 130, 50))
            d.text((257, 272), "1.0  AIMED ON THE ECE LINE", font=font(12), fill=(255, 255, 255))
        self.n += 1
        img.save(FRAMES / f"frame_{self.n:04d}.png")
        Image.fromarray(top).save(TOP / f"top_{self.n:04d}.png")
        self.last = img
        if self.first is None:
            self.first = img.copy()
            self.first.save(MEDIA / "setup.png")

    def video(self):
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(FRAMES / "frame_%04d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", "-preset", "veryfast",
                        "-movflags", "+faststart", str(MEDIA / "demo.mp4")], check=True)
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(TOP / "top_%04d.png"),
                        "-vf", "fps=8,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                        str(MEDIA / "demo_top.gif")], check=True)
        # a last clean 1.0 setup frame is not needed; keep the initial setup as the reference


def main():
    MEDIA.mkdir(parents=True, exist_ok=True)
    m = Media()
    result = run(str(TASK), steps=600, video=None, quiet=True, on_step=m.frame)
    m.video()
    print("task_score", result["task_score"], "success_step", result["success_step"], "frames", m.n)
    for name in ("setup.png", "demo.mp4", "demo_top.gif"):
        p = MEDIA / name
        print(name, p.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
