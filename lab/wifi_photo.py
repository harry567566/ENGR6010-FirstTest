"""Take one picture with the front camera of a REAL Go2 over Wi-Fi. READ-ONLY. The Wi-Fi twin of real_photo.py.

    .venv_wifi/bin/python lab/wifi_photo.py ap front.jpg

Over Wi-Fi the camera comes as a live video stream (like in the phone app), so this turns the stream on,
keeps one frame, and saves it as a JPEG.
"""
import asyncio
import sys

from wifi_common import STOP, connect, disconnect, fail, watch_for_stop

SKIP = 5  # the first frames of a stream can be grey while the video decoder starts


async def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    target, out_path = sys.argv[1], sys.argv[2]
    stop_event = asyncio.Event()
    watch_for_stop(stop_event)
    conn = await connect(target)
    got = asyncio.get_running_loop().create_future()

    async def on_track(track):
        n = 0
        while not got.done():
            frame = await track.recv()
            n += 1
            if n > SKIP and not got.done():
                got.set_result(frame)

    try:
        conn.video.add_track_callback(on_track)
        conn.video.switchVideoChannel(True)
        print("asking the front camera for video ...", flush=True)
        waiter = asyncio.ensure_future(stop_event.wait())
        done, _ = await asyncio.wait({got, waiter}, timeout=15, return_when=asyncio.FIRST_COMPLETED)
        waiter.cancel()
        if got not in done:
            conn.video.switchVideoChannel(False)
            if STOP["requested"]:
                fail(1, "stopped before a frame arrived", "还没收到画面就被停止了")
            fail(2, "no video frame within 15 s", "15 秒内没收到视频画面")
        frame = got.result()
        frame.to_image().save(out_path, "JPEG", quality=92)
        conn.video.switchVideoChannel(False)
        print(f"saved {out_path} ({frame.width} x {frame.height})")
    finally:
        await disconnect(conn)


if __name__ == "__main__":
    asyncio.run(main())
