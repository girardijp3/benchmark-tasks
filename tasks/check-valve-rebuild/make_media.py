"""Render the internal top-view media of the 1.0 MuJoCo demo (not PR media).

A downward, close-in top view of the valve work area, 640 x 360. The station's top camera frames the whole table,
so each rendered frame is cropped to the valve fixture and resized to 640x360: the cap, poppet, spring, pockets,
peg, buttons and both gripper tips are legible while the camera stays straight down. Writes setup.png, demo.mp4
and demo_top.gif into benchmark-media-check-valve-rebuild/. Every 3rd control step is captured.

Run: cd local_sim && PYTHONPATH=. .venv/bin/python ../benchmark-submission/tasks/check-valve-rebuild/make_media.py
"""
import os
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
from PIL import Image, ImageDraw, ImageFont

from mjkit.run import run

HERE = Path(__file__).resolve().parent
MEDIA = HERE.parents[2] / "benchmark-media-check-valve-rebuild"
FRAMES = MEDIA / "frames"
TOP = MEDIA / "top_frames"
TASK = HERE / "check_valve_rebuild.py"
W, H = 640, 360
EVERY = 3
# the station top camera render; crop a window around the valve at x~0.25,y~0 and resize to 640x360
SRC_W, SRC_H = 640, 360
CROP = (180, 130, 500, 310)   # left, upper, right, lower in the 640x360 render
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_R = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def font(size, bold=True):
    try:
        return ImageFont.truetype(FONT if bold else FONT_R, size)
    except OSError:
        return ImageFont.load_default()


class Media:
    def __init__(self):
        self.n = 0
        self.last = None
        for folder in (FRAMES, TOP):
            folder.mkdir(parents=True, exist_ok=True)
            for p in folder.glob("*.png"):
                p.unlink()

    def frame(self, sim, step, states, info):
        if step % EVERY and step != 1:
            return
        raw = Image.fromarray(sim.render("top_cam", width=SRC_W, height=SRC_H))
        top = raw.crop(CROP).resize((W, H), Image.LANCZOS)
        img = top.copy()
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, W, 24], fill=(18, 22, 28))
        d.text((8, 4), "top view: 20 mm brass spring check valve / rebuild + one-way flow", font=font(12),
               fill=(245, 245, 240))
        t = float(states.get("episode_time", 0.0))
        score = float(info.get("score", 0.0))
        d.rectangle([0, H - 46, W, H], fill=(18, 22, 28))
        d.text((8, H - 42), f"step {step:4d}   t {t:5.1f} s   score {score:.2f}", font=font(12),
               fill=(240, 240, 235))
        d.text((W - 92, H - 22), f"{t:5.1f} s", font=font(12), fill=(230, 230, 225))
        for k, (lab, flag) in enumerate((("body opened", "body_opened"), ("poppet + spring seated", "spring_done"),
                                         ("cap threaded + forward", "forward_done"),
                                         ("backflow sealed + hold", "reverse_done"))):
            yy = 30 + k * 17
            on = bool(states.get(flag)) and (k < 3 or score >= 1.0)
            col = (60, 190, 90) if on else (80, 84, 90)
            d.rectangle([8, yy, 22, yy + 13], fill=col)
            d.text((28, yy - 2), lab, font=font(11, False), fill=(245, 245, 240))
        self.n += 1
        img.save(FRAMES / f"frame_{self.n:04d}.png")
        top.save(TOP / f"top_{self.n:04d}.png")
        self.last = img
        if self.n == 1:
            img.save(MEDIA / "setup.png")

    def video(self):
        for _ in range(40):
            self.n += 1
            self.last.save(FRAMES / f"frame_{self.n:04d}.png")
            self.last.save(TOP / f"top_{self.n:04d}.png")
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(FRAMES / "frame_%04d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "25", "-preset", "veryfast",
                        "-movflags", "+faststart", str(MEDIA / "demo.mp4")], check=True)
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "20", "-i", str(TOP / "top_%04d.png"),
                        "-vf", "fps=6,scale=480:270:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                        str(MEDIA / "demo_top.gif")], check=True)


def main():
    MEDIA.mkdir(parents=True, exist_ok=True)
    m = Media()
    result = run(str(TASK), steps=1120, video=None, quiet=True, on_step=m.frame)
    m.video()
    print("task_score", result["task_score"], "success_step", result["success_step"], "frames", m.n)
    for name in ("setup.png", "demo.mp4", "demo_top.gif"):
        p = MEDIA / name
        print(name, p.stat().st_size, "bytes")


if __name__ == "__main__":
    main()
