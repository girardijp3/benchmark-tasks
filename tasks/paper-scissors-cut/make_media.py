"""Render the MuJoCo demo of paper-scissors-cut into the top-view media package.

Writes setup.png, demo.mp4 (with a held 1.0 ending) and demo_top.gif in benchmark-media-paper-scissors-cut/.
Run:  cd local_sim && PYTHONPATH=. .venv/bin/python ../benchmark-submission/tasks/paper-scissors-cut/make_media.py
"""
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from mjkit.run import run

HERE = Path(__file__).resolve().parent
MEDIA = HERE.parents[2] / "benchmark-media-paper-scissors-cut"
FRAMES = MEDIA / "frames"
TOP = MEDIA / "top_frames"
TASK = HERE / "paper_scissors_cut.py"

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
        self.inset_renderer = None
        self.cam = None
        self.n = 0
        self.last = None
        FRAMES.mkdir(parents=True, exist_ok=True)
        TOP.mkdir(parents=True, exist_ok=True)
        for p in FRAMES.glob("*.png"):
            p.unlink()
        for p in TOP.glob("*.png"):
            p.unlink()

    def _inset(self, sim):
        if self.inset_renderer is None:
            # initialise the station's visual options once (hides colliders)
            sim.render("top_cam", width=W, height=H)
            self.inset_renderer = mujoco.Renderer(sim.model, 360, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.lookat = np.array([0.465, -0.160, 0.7685])
            self.cam.distance = 0.62
            self.cam.azimuth = 0.0
            self.cam.elevation = -89.0     # MuJoCo's free camera looks from above at negative elevation
        self.inset_renderer.update_scene(sim.data, camera=self.cam, scene_option=sim._vopt)
        return cv2.resize(self.inset_renderer.render(), (INSET_W, INSET_H))

    def frame(self, sim, step, states, info):
        top = sim.render("top_cam", width=W, height=H)
        inset = self._inset(sim)
        img = Image.fromarray(top)
        d = ImageDraw.Draw(img)

        # header
        d.rectangle([0, 0, W, 24], fill=(18, 22, 28))
        d.text((8, 4), "top view: A4 sheet - 100 mm scissors cut", font=font(12), fill=(245, 245, 240))

        # close-up inset of the printed line and the blade
        x0, y0 = W - INSET_W - 10, 30
        img.paste(Image.fromarray(inset), (x0, y0))
        d.rectangle([x0 - 2, y0 - 2, x0 + INSET_W + 1, y0 + INSET_H + 1], outline=(245, 245, 240), width=2)
        d.rectangle([x0 - 2, y0 - 20, x0 + INSET_W + 1, y0 - 2], fill=(18, 22, 28))
        d.text((x0 + 4, y0 - 18), "cut line & blade (close-up)", font=font(11), fill=(240, 240, 235))

        # score ladder
        score = float(info.get("score", 0.0))
        for k, (lab, on) in enumerate((("scissors taken", score >= 0.25),
                                       ("sheet held, start", score >= 0.5),
                                       ("line cut", score >= 0.75),
                                       ("1.0 hold", score >= 1.0))):
            yy = 32 + k * 19
            d.rectangle([8, yy, 20, yy + 13], fill=(60, 190, 90) if on else (80, 84, 90))
            d.text((26, yy - 1), lab, font=font(11, False), fill=(240, 240, 235))

        # bottom telemetry + cut progress bar
        d.rectangle([0, H - 54, W, H], fill=(18, 22, 28))
        cut = float(states.get("cut_front", -1.0)) - 0.360
        cut = min(max(cut, 0.0), 0.210) * 1000.0
        dev = float(states.get("cut_dev", 0.0)) * 1000.0
        blade = float(states.get("blade_open", 0.0)) * 180.0 / np.pi
        press = bool(states.get("pressed", False))
        grip = bool(states.get("on_handles", False))
        tray = bool(states.get("offcut_in_tray", False))
        t = float(states.get("episode_time", 0.0))
        d.text((8, H - 51), f"cut {cut:5.0f}/210 mm   dev {dev:4.1f} mm   blade {blade:4.1f} deg",
               font=font(12), fill=(240, 240, 235))
        d.text((8, H - 33), f"press {str(press):5s}  grip {str(grip):5s}  offcut in tray {str(tray):5s}",
               font=font(11, False), fill=(225, 225, 220))
        d.rectangle([8, H - 14, 208, H - 9], outline=(160, 160, 160))
        d.rectangle([8, H - 14, 8 + int(200 * cut / 210.0), H - 9], fill=(60, 190, 90))
        d.text((W - 74, H - 22), f"{t:5.1f} s", font=font(12), fill=(235, 235, 230))
        if score >= 1.0:
            d.rectangle([240, 96, 480, 126], fill=(20, 130, 50))
            d.text((248, 101), "1.0  CUT COMPLETE / OFFCUT TRAYED", font=font(11), fill=(255, 255, 255))

        self.n += 1
        img.save(FRAMES / f"frame_{self.n:04d}.png")
        if self.n == 1:
            img.save(MEDIA / "setup.png")

        # pure top view (light overlay) for the compact gif
        tg = Image.fromarray(top)
        td = ImageDraw.Draw(tg)
        td.rectangle([0, 0, W, 20], fill=(18, 22, 28))
        td.text((6, 2), "top view - 100 mm A4 cut", font=font(10), fill=(245, 245, 240))
        td.rectangle([0, H - 22, W, H], fill=(18, 22, 28))
        td.text((6, H - 20), f"cut {cut:5.0f}/210 mm   score {score:3.1f}   {t:5.1f} s",
                font=font(10, False), fill=(240, 240, 235))
        td.rectangle([W - 74, 4, W - 8, 16], outline=(80, 84, 90))
        td.rectangle([W - 74, 4, W - 74 + int(66 * score), 16], fill=(60, 190, 90))
        tg.save(TOP / f"top_{self.n:04d}.png")
        self.last = img

    def video(self):
        # hold the final 1.0 frame for 2 s (40 frames at 20 fps); the episode itself ends at the success step
        last = FRAMES / f"frame_{self.n:04d}.png"
        for i in range(1, 41):
            (FRAMES / f"frame_{self.n + i:04d}.png").write_bytes(last.read_bytes())
        last_top = TOP / f"top_{self.n:04d}.png"
        for i in range(1, 21):
            (TOP / f"top_{self.n + i:04d}.png").write_bytes(last_top.read_bytes())
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(FRAMES / "frame_%04d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", "-preset", "veryfast",
                        "-movflags", "+faststart", str(MEDIA / "demo.mp4")], check=True)
        # the PNGs are one per 20 Hz control step; keep real time by declaring 20 fps on input and decimating to 5 fps
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(TOP / "top_%04d.png"),
                        "-vf", "fps=5,split[a][b];[a]palettegen[p];[b][p]paletteuse", "-loop", "0",
                        str(MEDIA / "demo_top.gif")], check=True)


def main():
    MEDIA.mkdir(parents=True, exist_ok=True)
    m = Media()
    result = run(str(TASK), steps=620, video=None, quiet=True, on_step=m.frame)
    m.video()
    print("task_score", result["task_score"], "success_step", result["success_step"], "frames", m.n)
    for name in ("setup.png", "demo.mp4", "demo_top.gif"):
        p = MEDIA / name
        print(name, p.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
