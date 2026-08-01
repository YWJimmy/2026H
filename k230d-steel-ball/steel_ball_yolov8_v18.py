"""CanMV K230D v1.8 / nncase 2.11.0 小钢球 YOLOv8 部署脚本。

KModel 有两个输出：boxes ``[1,4,2100]`` 和 scores ``[1,1,2100]``。
检测坐标统一映射到 1280x960：

    SB,1,x1,y1,x2,y2,cx,cy,score_milli
    SB,0,0,0,0,0,0,0,0
"""

from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.PipeLine import PipeLine, ScopedTiming
from media.sensor import Sensor
import aidemo
import gc
import nncase_runtime as nn
import os
import sys
import ulab.numpy as np


KMODEL_PATH = "/sdcard/steel_ball_yolov8n_320_nncase211_w8a16.kmodel"
MODEL_INPUT_SIZE = [320, 320]
AI_FRAME_SIZE = [320, 240]
OUTPUT_COORD_SIZE = [1280, 960]
CONFIDENCE_THRESHOLD = 0.08
NMS_THRESHOLD = 0.45
MAX_BOXES = 10
DISPLAY_MODE = "lcd"


def remove_aidemo_letterbox_offset(box):
    """从 aidemo v1.8 的 xywh 输出中消除居中 AI2D 填充偏移。"""
    source_w, source_h = AI_FRAME_SIZE
    model_w, model_h = MODEL_INPUT_SIZE
    output_w, output_h = OUTPUT_COORD_SIZE
    preprocess_scale = min(
        float(model_w) / source_w,
        float(model_h) / source_h,
    )
    resized_w = int(round(source_w * preprocess_scale))
    resized_h = int(round(source_h * preprocess_scale))
    pad_left = int(round((model_w - resized_w) / 2 - 0.1))
    pad_top = int(round((model_h - resized_h) / 2 - 0.1))
    output_scale_x = float(output_w) / source_w
    output_scale_y = float(output_h) / source_h
    offset_x = pad_left / preprocess_scale * output_scale_x
    offset_y = pad_top / preprocess_scale * output_scale_y
    x, y, width, height = box
    return [x - offset_x, y - offset_y, width, height]


class SteelBallYoloV8(AIBase):
    def __init__(self, kmodel_path, display_size, debug_mode=0):
        AIBase.__init__(
            self,
            kmodel_path,
            MODEL_INPUT_SIZE,
            AI_FRAME_SIZE,
            debug_mode,
        )
        self.model_input_size = MODEL_INPUT_SIZE
        self.rgb888p_size = AI_FRAME_SIZE
        self.display_size = display_size
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            size = input_image_size if input_image_size else self.rgb888p_size
            src_w, src_h = size
            dst_w, dst_h = self.model_input_size
            scale = min(float(dst_w) / src_w, float(dst_h) / src_h)
            resized_w = int(round(src_w * scale))
            resized_h = int(round(src_h * scale))
            left = int(round((dst_w - resized_w) / 2 - 0.1))
            right = int(round((dst_w - resized_w) / 2 + 0.1))
            top = int(round((dst_h - resized_h) / 2 - 0.1))
            bottom = int(round((dst_h - resized_h) / 2 + 0.1))
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [114, 114, 114],
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, src_h, src_w],
                [1, 3, dst_h, dst_w],
            )

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            if len(results) == 2:
                boxes = results[0].reshape((4, 2100))
                scores = results[1].reshape((1, 2100))
                prediction = np.concatenate((boxes, scores), axis=0).transpose()
            elif len(results) == 1:
                prediction = results[0][0].transpose()
            else:
                raise RuntimeError("unexpected KModel output count: %d" % len(results))
            return aidemo.yolov8_det_postprocess(
                prediction.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [OUTPUT_COORD_SIZE[1], OUTPUT_COORD_SIZE[0]],
                1,
                CONFIDENCE_THRESHOLD,
                NMS_THRESHOLD,
                MAX_BOXES,
            )

    def best_detection(self, result):
        if not result or not result[0]:
            return None
        best_index = 0
        for index in range(1, len(result[0])):
            if result[2][index] > result[2][best_index]:
                best_index = index
        x, y, width, height = remove_aidemo_letterbox_offset(
            result[0][best_index]
        )
        x1 = max(0, int(round(x)))
        y1 = max(0, int(round(y)))
        x2 = min(OUTPUT_COORD_SIZE[0] - 1, int(round(x + width)))
        y2 = min(OUTPUT_COORD_SIZE[1] - 1, int(round(y + height)))
        return [x1, y1, x2, y2, float(result[2][best_index])]

    def draw_result(self, pipeline, detection):
        pipeline.osd_img.clear()
        if detection is None:
            return
        x1, y1, x2, y2, score = detection
        display_w, display_h = self.display_size
        dx1 = int(x1 * display_w / OUTPUT_COORD_SIZE[0])
        dy1 = int(y1 * display_h / OUTPUT_COORD_SIZE[1])
        dx2 = int(x2 * display_w / OUTPUT_COORD_SIZE[0])
        dy2 = int(y2 * display_h / OUTPUT_COORD_SIZE[1])
        dcx = (dx1 + dx2) // 2
        dcy = (dy1 + dy2) // 2
        color = (255, 0, 255, 0)
        pipeline.osd_img.draw_rectangle(
            dx1,
            dy1,
            dx2 - dx1,
            dy2 - dy1,
            color=color,
            thickness=3,
        )
        pipeline.osd_img.draw_line(
            dcx - 10, dcy, dcx + 10, dcy, color=color, thickness=2
        )
        pipeline.osd_img.draw_line(
            dcx, dcy - 10, dcx, dcy + 10, color=color, thickness=2
        )
        pipeline.osd_img.draw_string_advanced(
            dx1,
            max(0, dy1 - 28),
            24,
            "steel_ball %.2f" % score,
            color=color,
        )


def report_detection(detection):
    if detection is None:
        print("SB,0,0,0,0,0,0,0,0")
        return
    x1, y1, x2, y2, score = detection
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    print(
        "SB,1,%d,%d,%d,%d,%d,%d,%d"
        % (x1, y1, x2, y2, center_x, center_y, int(score * 1000))
    )


def main():
    pipeline = None
    detector = None
    try:
        sensor = Sensor(width=1280, height=960)
        pipeline = PipeLine(
            rgb888p_size=AI_FRAME_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=None,
        )
        pipeline.create(sensor=sensor)
        detector = SteelBallYoloV8(
            KMODEL_PATH,
            pipeline.get_display_size(),
            debug_mode=0,
        )
        detector.config_preprocess()
        while True:
            os.exitpoint()
            frame = pipeline.get_frame()
            result = detector.run(frame)
            detection = detector.best_detection(result)
            report_detection(detection)
            detector.draw_result(pipeline, detection)
            pipeline.show_image()
            gc.collect()
    except Exception as exception:
        sys.print_exception(exception)
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        gc.collect()
        nn.shrink_memory_pool()


main()
