# Gamepad DDS Demo

这个 demo 用来验证手柄 `wireless_remote[40]` 的发送和接收打印流程。

## 编译

推荐使用脚本：

```bash
./scripts/build_demo.sh
```

如果想编译后顺便运行 demo 解码测试：

```bash
./scripts/build_demo.sh --test
```

脚本会编译：

- `gamepad_dds_sender`
- `gamepad_dds_receiver`
- `unitree_g1_gamepad_test`

## 启动

开两个终端。

终端 1，启动接收端：

```bash
./build-g1-only/gamepad_dds_receiver config/g1.sample.yaml
```

终端 2，插上手柄后启动发送端：

```bash
./build-g1-only/gamepad_dds_sender config/g1.sample.yaml 5 0
```

参数说明：

- `config/g1.sample.yaml`：配置文件
- `5`：发送频率 Hz
- `0`：SDL 手柄 index

## SDL 手柄映射

如果 raw SDL events 能看到 trigger，但 sender 表格里的 `lt/rt` 不动，通常是 SDL
GameController mapping 错。配置文件里可以指定 mapping 数据库：

```yaml
gamepad:
  mapping_path: docs/joy_stick/include/hardware/gamecontrollerdb.txt
```

`gamepad_dds_sender` 会在打开手柄前加载这个文件。

映射文件默认仅含注释；需要额外设备映射时，从 [SDL_GameControllerDB](https://github.com/gabomdq/SDL_GameControllerDB) 获取与设备匹配的条目。

## 观察输出

发送端会打印加密/打包前的值：

```text
[pre_encrypt] ...
sdl_axis ...
raw_axis ...
filtered_axis ...
```

接收端会打印 DDS 中 `wireless_remote` 解码后的值：

```text
Axis  lx:... ly:... rx:... ry:... lt:... rt:...
Buttons A:... B:... X:... Y:...
Custom Buttons ...
```

正常映射应满足：

- 左摇杆改变 `lx/ly`
- 右摇杆改变 `rx/ry`
- 左扳机改变 `lt`
- 右扳机改变 `rt`

## 只运行测试

```bash
ctest --test-dir build-g1-only -R '^unitree_g1_gamepad_test$' --output-on-failure
```
