#!/usr/bin/env python3
import sys
import os
import time
from collections import deque

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstBase", "1.0")
gi.require_version("GObject", "2.0")
gi.require_version("GLib", "2.0")

from gi.repository import Gst, GObject, GLib

try:
    import pyds
except ImportError:
    print("Không import được pyds. Hãy đảm bảo đã cài DeepStream Python bindings.")
    sys.exit(1)

# ===================== CONFIG =====================

DS_ROOT = "/opt/nvidia/deepstream/deepstream-6.1"

NVPARAM_CONFIG = "/home/asiclab/peoplenet_demo/models/nvinfer_config.txt"

TRACKER_LIB_PATH = os.path.join(DS_ROOT, "lib", "libnvds_nvmultiobjecttracker.so")
TRACKER_CONFIG_PATH = os.path.join(
    DS_ROOT, "samples", "configs", "deepstream-app", "config_tracker_IOU.yml"
)

STREAMMUX_WIDTH = 960
STREAMMUX_HEIGHT = 544

ENABLE_TRACKER = True

# ===================== FPS & CONF SMOOTHING =====================

_frame_counter = 0
_last_fps_ts = time.time()
_fps_history = deque(maxlen=30)
_conf_history = deque(maxlen=60)


def update_fps():
    global _frame_counter, _last_fps_ts
    _frame_counter += 1
    now = time.time()
    dt = now - _last_fps_ts
    fps_smooth = 0.0

    if dt >= 1.0:
        fps = _frame_counter / dt
        _fps_history.append(fps)
        _frame_counter = 0
        _last_fps_ts = now

    if len(_fps_history) > 0:
        fps_smooth = sum(_fps_history) / len(_fps_history)
    return fps_smooth


def update_conf(avg_conf_frame):
    if avg_conf_frame is not None:
        _conf_history.append(avg_conf_frame)
    if len(_conf_history) == 0:
        return 0.0
    return sum(_conf_history) / len(_conf_history)


# ===================== PAD PROBE (OSD) =====================

def osd_sink_pad_buffer_probe(pad, info, u_data):
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        return Gst.PadProbeReturn.OK

    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    if not batch_meta:
        return Gst.PadProbeReturn.OK

    l_frame = batch_meta.frame_meta_list

    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        num_obj = frame_meta.num_obj_meta
        l_obj = frame_meta.obj_meta_list

        conf_sum = 0.0
        conf_cnt = 0

        while l_obj is not None:
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break

            # ----- STYLE CHO BOX -----
            rect_params = obj_meta.rect_params
            rect_params.border_width = 3
            rect_params.border_color.set(0.0, 1.0, 0.0, 1.0)  # RGBA

            text_params = obj_meta.text_params
            if text_params.display_text:
                text_params.display_text = f"{text_params.display_text} ({obj_meta.confidence:.2f})"
            else:
                text_params.display_text = f"obj {obj_meta.class_id} ({obj_meta.confidence:.2f})"

            text_params.font_params.font_name = "Sans"
            text_params.font_params.font_size = 12
            text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

            # Nền mờ phía sau chữ (tùy version pyds)
            text_params.set_bg_clr = 1
            try:
                # DS 6.x thường có text_bg_clr
                text_params.text_bg_clr.set(0.0, 0.0, 0.0, 0.4)
            except AttributeError:
                # Nếu version này không có text_bg_clr thì bỏ qua
                pass

            conf_sum += max(0.0, min(1.0, float(obj_meta.confidence)))
            conf_cnt += 1

            l_obj = l_obj.next

        avg_conf_frame = None
        if conf_cnt > 0:
            avg_conf_frame = conf_sum / conf_cnt

        fps_smooth = update_fps()
        conf_smooth = update_conf(avg_conf_frame)

        # ----- OVERLAY FPS + CONF + OBJECT COUNT -----
        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1

        text_params = display_meta.text_params[0]
        text_params.display_text = (
            f"FPS: {fps_smooth:5.1f}  |  "
            f"Avg Conf: {conf_smooth:4.2f}  |  Objects: {num_obj}"
        )
        text_params.x_offset = 10
        text_params.y_offset = 20
        text_params.font_params.font_name = "Sans"
        text_params.font_params.font_size = 16
        text_params.font_params.font_color.set(1.0, 1.0, 0.0, 1.0)
        text_params.set_bg_clr = 1
        try:
            text_params.text_bg_clr.set(0.0, 0.0, 0.0, 0.6)
        except AttributeError:
            pass

        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        l_frame = l_frame.next

    return Gst.PadProbeReturn.OK


# ===================== PIPELINE CREATION =====================

def create_pipeline():
    Gst.init(None)

    pipeline = Gst.Pipeline.new("peoplenet-pipeline")
    if not pipeline:
        print("Không tạo được pipeline")
        return None

    # ---- Source camera ----
    source = Gst.ElementFactory.make("v4l2src", "usb-cam-source")
    caps_src = Gst.ElementFactory.make("capsfilter", "caps_src")
    jpegdec = Gst.ElementFactory.make("jpegdec", "jpeg-decoder")
    nvconv1 = Gst.ElementFactory.make("nvvideoconvert", "nvconv1")
    caps_nvmm = Gst.ElementFactory.make("capsfilter", "caps_nvmm")

    streammux = Gst.ElementFactory.make("nvstreammux", "streammux")
    pgie = Gst.ElementFactory.make("nvinfer", "primary-infer")

    tracker = None
    if ENABLE_TRACKER:
        tracker = Gst.ElementFactory.make("nvtracker", "tracker")

    nvconv2 = Gst.ElementFactory.make("nvvideoconvert", "nvconv2")
    caps_rgba = Gst.ElementFactory.make("capsfilter", "caps_rgba")
    nvdsosd = Gst.ElementFactory.make("nvdsosd", "nvdsosd")
    nvegltransform = Gst.ElementFactory.make("nvegltransform", "nvegltransform")
    sink = Gst.ElementFactory.make("nveglglessink", "egl_sink")

    if not all([source, caps_src, jpegdec, nvconv1, caps_nvmm,
                streammux, pgie, nvconv2, caps_rgba,
                nvdsosd, nvegltransform, sink]):
        print("Không tạo đủ element GStreamer. Kiểm tra DeepStream / GStreamer cài đặt.")
        return None

    if ENABLE_TRACKER and not tracker:
        print("Không tạo được nvtracker, sẽ chạy không tracker.")
        ENABLE_TRACKER_LOCAL = False
    else:
        ENABLE_TRACKER_LOCAL = ENABLE_TRACKER

    # ---- Set properties ----

    # Camera
    source.set_property("device", "/dev/video0")
    source.set_property("io-mode", 2)  # userptr

    caps_src.set_property(
        "caps",
        Gst.Caps.from_string(
            "image/jpeg,width=640,height=480,framerate=30/1,format=MJPG"
        ),
    )

    caps_nvmm.set_property(
        "caps",
        Gst.Caps.from_string("video/x-raw(memory:NVMM),format=NV12"),
    )

    # streammux
    streammux.set_property("live-source", 1)
    streammux.set_property("batch-size", 1)
    streammux.set_property("width", STREAMMUX_WIDTH)
    streammux.set_property("height", STREAMMUX_HEIGHT)
    streammux.set_property("batched-push-timeout", 40000)

    # nvinfer
    pgie.set_property("config-file-path", NVPARAM_CONFIG)
    pgie.set_property("batch-size", 1)

    # tracker
    if ENABLE_TRACKER_LOCAL:
        tracker.set_property("ll-lib-file", TRACKER_LIB_PATH)
        tracker.set_property("ll-config-file", TRACKER_CONFIG_PATH)
        tracker.set_property("enable-batch-process", 1)
        tracker.set_property("gpu_id", 0)

    # RGBA for nvdsosd
    caps_rgba.set_property(
        "caps",
        Gst.Caps.from_string("video/x-raw(memory:NVMM),format=RGBA"),
    )

    nvdsosd.set_property("process-mode", 1)
    nvdsosd.set_property("display-text", 1)

    sink.set_property("sync", False)

    # ---- Add elements ----
    pipeline.add(source)
    pipeline.add(caps_src)
    pipeline.add(jpegdec)
    pipeline.add(nvconv1)
    pipeline.add(caps_nvmm)
    pipeline.add(streammux)
    pipeline.add(pgie)
    if ENABLE_TRACKER_LOCAL:
        pipeline.add(tracker)
    pipeline.add(nvconv2)
    pipeline.add(caps_rgba)
    pipeline.add(nvdsosd)
    pipeline.add(nvegltransform)
    pipeline.add(sink)

    # ---- Link camera -> caps_nvmm ----
    if not source.link(caps_src):
        print("Không link được source -> caps_src")
        return None
    if not caps_src.link(jpegdec):
        print("Không link được caps_src -> jpegdec")
        return None
    if not jpegdec.link(nvconv1):
        print("Không link được jpegdec -> nvconv1")
        return None
    if not nvconv1.link(caps_nvmm):
        print("Không link được nvconv1 -> caps_nvmm")
        return None

    # ---- caps_nvmm -> streammux ----
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        print("Không lấy được sink_0 pad của streammux")
        return None
    srcpad = caps_nvmm.get_static_pad("src")
    if not srcpad:
        print("Không lấy được src pad của caps_nvmm")
        return None
    if srcpad.link(sinkpad) != Gst.PadLinkReturn.OK:
        print("Không link được caps_nvmm -> streammux (pad link)")
        return None

    # ---- streammux -> pgie -> tracker? -> nvdsosd -> sink ----
    if not streammux.link(pgie):
        print("Không link được streammux -> nvinfer")
        return None

    if ENABLE_TRACKER_LOCAL:
        if not pgie.link(tracker):
            print("Không link được nvinfer -> nvtracker")
            return None
        if not tracker.link(nvconv2):
            print("Không link được nvtracker -> nvconv2")
            return None
    else:
        if not pgie.link(nvconv2):
            print("Không link được nvinfer -> nvconv2 (không tracker)")
            return None

    if not nvconv2.link(caps_rgba):
        print("Không link được nvconv2 -> caps_rgba")
        return None
    if not caps_rgba.link(nvdsosd):
        print("Không link được caps_rgba -> nvdsosd")
        return None
    if not nvdsosd.link(nvegltransform):
        print("Không link được nvdsosd -> nvegltransform")
        return None
    if not nvegltransform.link(sink):
        print("Không link được nvegltransform -> sink")
        return None

    # ---- Pad probe ----
    osd_sink_pad = nvdsosd.get_static_pad("sink")
    if not osd_sink_pad:
        print("Không lấy được sink pad của nvdsosd")
        return None

    osd_sink_pad.add_probe(
        Gst.PadProbeType.BUFFER,
        osd_sink_pad_buffer_probe,
        None,
    )

    return pipeline


# ===================== MAIN =====================

def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("Nhận EOS, dừng pipeline...")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print("Lỗi từ bus:", err, debug)
        loop.quit()
    return True


def main():
    GObject.threads_init()
    Gst.init(None)

    pipeline = create_pipeline()
    if not pipeline:
        print("Không tạo được pipeline, thoát.")
        sys.exit(1)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    print("Khởi động pipeline (PeopleNet + {}tracker)... Nhấn Ctrl+C để thoát.".format(
        "" if ENABLE_TRACKER else "NO-"
    ))

    pipeline.set_state(Gst.State.PLAYING)

    try:
        loop.run()
    except KeyboardInterrupt:
        print("Nhận Ctrl+C, dừng pipeline...")

    pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
