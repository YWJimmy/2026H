"""CanMV K230D：不启动摄像头和 LCD，诊断小钢球 KModel 加载与推理。"""

import gc
import nncase_runtime as nn
import os
import sys
import ulab.numpy as np


KMODEL_PATH = "/sdcard/steel_ball_yolov8n_320_nncase211_w8a16.kmodel"


def print_free_memory(stage):
    try:
        print(
            "[DEBUG-K230-STEEL-KMODEL] stage=%s mem_free=%d"
            % (stage, gc.mem_free())
        )
    except Exception:
        print("[DEBUG-K230-STEEL-KMODEL] stage=%s" % stage)


def main():
    model = None
    input_array = None
    input_tensor = None
    try:
        print_free_memory("start")
        stat = os.stat(KMODEL_PATH)
        print(
            "[DEBUG-K230-STEEL-KMODEL] stage=file_ok size=%d"
            % stat[6]
        )

        print_free_memory("before_load")
        model = nn.kpu()
        model.load_kmodel(KMODEL_PATH)
        print_free_memory("load_ok")

        print("[DEBUG-K230-STEEL-KMODEL] inputs=%d" % model.inputs_size())
        for index in range(model.inputs_size()):
            print(
                "[DEBUG-K230-STEEL-KMODEL] input_desc[%d]=%s"
                % (index, model.inputs_desc(index))
            )
        print("[DEBUG-K230-STEEL-KMODEL] outputs=%d" % model.outputs_size())
        for index in range(model.outputs_size()):
            print(
                "[DEBUG-K230-STEEL-KMODEL] output_desc[%d]=%s"
                % (index, model.outputs_desc(index))
            )

        print_free_memory("create_input_begin")
        input_array = np.zeros((1, 3, 320, 320), dtype=np.uint8)
        input_tensor = nn.from_numpy(input_array)
        print_free_memory("create_input_ready")

        print_free_memory("set_input_begin")
        model.set_input_tensor(0, input_tensor)
        print_free_memory("set_input_ready")

        print_free_memory("run_begin")
        model.run()
        print_free_memory("run_ready")

        print_free_memory("get_outputs_begin")
        for index in range(model.outputs_size()):
            output_data = model.get_output_tensor(index)
            output_array = output_data.to_numpy()
            print(
                "[DEBUG-K230-STEEL-KMODEL] output[%d] shape=%s dtype=%s"
                % (index, output_array.shape, output_array.dtype)
            )
            del output_array
            del output_data
        print_free_memory("get_outputs_ready")
    except Exception as exception:
        print("[DEBUG-K230-STEEL-KMODEL] stage=exception")
        sys.print_exception(exception)
    finally:
        if input_tensor is not None:
            del input_tensor
        if input_array is not None:
            del input_array
        if model is not None:
            del model
        gc.collect()
        nn.shrink_memory_pool()
        print_free_memory("cleanup")


main()
