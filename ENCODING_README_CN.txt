中文工程包编码说明
==================

UTF8_SAFE 版本：
- .c/.h/.s/.py/.uvprojx/.uvoptx/.ioc/.sct 使用 UTF-8 无 BOM或保持原始 ASCII；
- 本说明和核查报告使用 UTF-8 BOM；
- ZIP 文件名使用 UTF-8 标志。

Keil_GB18030 版本：
- 本补丁内修改的 C/H 文件使用 GB18030（这些修改文件目前主要为 ASCII，内容等价）；
- .uvprojx/.uvoptx/.ioc/.sct 不添加 BOM；
- Python 检查脚本使用 UTF-8 无 BOM；
- 本说明和核查报告使用 UTF-8 BOM；
- ZIP 文件名使用 UTF-8 标志。

禁止把 UTF-8 BOM 批量加入 Keil 工程配置、CubeMX 配置或 C/H/S 源码。
