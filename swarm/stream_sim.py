"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Make a copy of a recording whose video has been through the sim's live video
stream, at a given bitrate: what a companion (X) receives live. Replaying it
measures how much stream compression costs a tracker without flying again.

    python stream_sim.py RUN_DIR --bitrate 8000 [--drone Drone1]

The sim streams the GimbalCam (payload_video.RtspStreamer) as H.264 at a
steady 30 frames/s, repeating the newest rendered frame, with h264_nvenc
(preset p1, low latency, a keyframe every second) or libx264 (ultrafast)
when there is no NVIDIA GPU. --bitrate 0 leaves the bitrate to the encoder,
as the sim did before it had a setting (about 2.2 Mbit/s with h264_nvenc at
1080p).

Here the recording's frames are repeated the same way:
- the unique rate is the recording's own, from its frame times
- the result is encoded with the same settings, then decoded
- the first copy of each recorded frame is kept, stored near-lossless

So the new video has exactly the recording's frames, in order, and the
recording's frame list and true boxes still apply. Written to
RUN_DIR_stream<kbps>k/ (or _streamdefault/) with the recording's other files
copied in.
"""

import argparse
import csv
import json
import os
import shutil
import subprocess

import cv2

FPS = 30


def encoder_args(bitrate_kbps):
    encoders = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"],
                              capture_output=True, text=True).stdout
    if "h264_nvenc" in encoders:
        codec = ["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ll", "-zerolatency", "1"]
    else:
        codec = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency"]
    if bitrate_kbps:
        codec += ["-b:v", f"{bitrate_kbps}k", "-maxrate", f"{bitrate_kbps}k",
                  "-bufsize", f"{2 * bitrate_kbps}k"]
    return codec + ["-g", str(FPS), "-bf", "0", "-pix_fmt", "yuv420p"]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir")
    parser.add_argument("--bitrate", type=int, required=True,
                        help="kbit/s; 0 = the encoder's default")
    parser.add_argument("--drone", default="Drone1")
    args = parser.parse_args()

    run = os.path.normpath(args.run_dir)
    tag = f"stream{args.bitrate}k" if args.bitrate else "streamdefault"
    out = f"{run}_{tag}"
    os.makedirs(out, exist_ok=True)
    for name in os.listdir(run):
        path = os.path.join(run, name)
        if os.path.isfile(path) and not name.endswith(".mp4"):
            shutil.copy(path, out)

    with open(os.path.join(run, f"frames_{args.drone}.csv")) as f:
        times = [float(r["wall_time"]) for r in csv.DictReader(f)]
    unique_fps = (len(times) - 1) / (times[-1] - times[0])

    src = cv2.VideoCapture(os.path.join(run, f"gimbal_{args.drone}.mp4"))
    w, h = int(src.get(cv2.CAP_PROP_FRAME_WIDTH)), int(src.get(cv2.CAP_PROP_FRAME_HEIGHT))
    stream_path = os.path.join(out, "stream.mp4")
    enc = subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
         "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-framerate", str(FPS), "-i", "-",
         *encoder_args(args.bitrate), stream_path], stdin=subprocess.PIPE)
    # Output frame k shows the newest recorded frame by then: floor(k * unique / 30)
    first_copy = []  # output index of each recorded frame's first copy
    k, i, frame = 0, -1, None
    n = len(times)
    while True:
        want = int(k * unique_fps / FPS)
        if want >= n:
            break
        while i < want:
            ok, frame = src.read()
            if not ok:
                break
            i += 1
            first_copy.append(k)
        if i < want:
            break
        enc.stdin.write(frame.tobytes())
        k += 1
    enc.stdin.close()
    enc.wait()
    src.release()

    # Decode the stream; keep the first copy of each recorded frame
    keep = set(first_copy)
    dec = cv2.VideoCapture(stream_path)
    writer = subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
         "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-framerate", str(FPS), "-i", "-",
         "-c:v", "libx264", "-preset", "veryfast", "-crf", "10", "-pix_fmt", "yuv420p",
         os.path.join(out, f"gimbal_{args.drone}.mp4")], stdin=subprocess.PIPE)
    kept, k = 0, 0
    while True:
        ok, img = dec.read()
        if not ok:
            break
        if k in keep:
            writer.stdin.write(img.tobytes())
            kept += 1
        k += 1
    writer.stdin.close()
    writer.wait()
    dec.release()
    stream_bits = os.path.getsize(stream_path) * 8 / (k / FPS) if k else 0
    os.remove(stream_path)

    meta_path = os.path.join(out, "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    meta["stream_sim"] = {"bitrate_kbps": args.bitrate or None,
                          "measured_kbps": round(stream_bits / 1000),
                          "unique_fps": round(unique_fps, 2), "frames": kept,
                          "recorded_frames": len(first_copy)}
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"{out}: {kept} of {len(first_copy)} frames through a "
          f"{round(stream_bits / 1000)} kbit/s stream ({unique_fps:.1f} unique frames/s)")


if __name__ == "__main__":
    main()
