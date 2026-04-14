# DDS 结构说明

## 3. 当前 DDS 层的核心数据流

### 3.1 命令流（DDS -> Core）

```text
外部 DDS 网络
    |
    v
command_topic (LowCmd)
    |
    v
runtime->command_reader
    |
    v
DdsCommandIngress::on_data_available()
    |
    v
translate_lowcmd_to_hardware_command()
    |
    v
core::CommandFrame
    |
    v
BridgeRunner::submit_latest_command()
```

### 3.2 状态流（Core -> DDS）

```text
BridgeRunner
    |
    v
core::StateFrame
    |
    v
DdsStateEgress::publish()
    |
    +--> translate_hardware_state_to_lowstate()
    |
    +--> translate_hardware_state_to_bridge_status()
    |
    +--> runtime->state_writer->write(...)
    |
    +--> runtime->status_writer->write(...)
```

---

## 4. `runtime` 的意义是什么

`DdsRuntime` 可以理解为：

> **整个 DDS 通信运行环境的资源包**

它保存的是 Fast DDS 世界里的对象，而不是 bridge 自己发明的抽象对象。

例如：

- `participant`：当前 DDS 节点
- `publisher`：创建 DataWriter 的上层对象
- `subscriber`：创建 DataReader 的上层对象
- `command_topic/state_topic/status_topic`：话题
- `command_reader`：接收命令
- `state_writer/status_writer`：发送状态

也就是说：

- `runtime` 负责“把公路修好”
- `ingress/egress` 负责“在公路上运数据”

---

## 5. Fast DDS 标准层级关系

当前代码使用的是 Fast DDS 的标准实体层级：

```text
DomainParticipant
    ├── Publisher
    │      └── DataWriter
    └── Subscriber
           └── DataReader
```

### 5.1 `Subscriber` 是什么

`Subscriber` 是 DDS 的正式实体，用来：

- 创建 `DataReader`
- 管理一组 `DataReader`

例如：

```cpp
runtime->subscriber->create_datareader(...)
```

### 5.2 `Listener` 是什么

`Listener` 不是 DDS 通信实体，而是一个**事件回调对象**。

例如：

```cpp
class DdsCommandIngress : public DataReaderListener
```

它的意义是：

- 当 `DataReader` 收到新数据时
- Fast DDS 自动回调 `on_data_available()`

所以：

- `Subscriber` 是“创建 reader 的实体”
- `Listener` 是“reader 收到数据后通知谁”的机制

它们不是同一层东西。

---

## 6. `DdsCommandIngress::start()` 在做什么

这个函数的作用是：

> **把 ingress 自己注册成 `command_reader` 的 listener，并保存 core 的回调入口。**

逻辑如下：

```cpp
command_sink_ = &command_sink;
event_sink_ = &event_sink;
dependencies_.runtime->command_reader->set_listener(this);
```

意思是：

1. 记住命令应该交给谁（`command_sink_`）
2. 记住 transport fault 应该上报给谁（`event_sink_`）
3. 把当前对象注册给 Fast DDS reader

注册之后，只要 DDS 收到新命令，Fast DDS 就会调用：

```cpp
on_data_available(...)
```

然后 ingress 再负责把 DDS 数据翻译成 core 数据。

---

## 8. 当前结构里还值得注意的点

### 8.1 `dds_facade.hpp` 名字略有历史残留

这个文件里当前只有 `DdsAssembly`，所以它更像 assembly 定义，而不是典型意义上的 facade。

### 8.2 `DdsCommandIngress` 的 listener 生命周期要注意

`start()` 会：

```cpp
set_listener(this)
```

`stop()` 会：

```cpp
set_listener(nullptr)
```

因此在后续维护中要注意：

- 如果 ingress 被销毁前没有正确 `stop()`
- 那么 reader 中可能还残留一个失效 listener 指针

这是后续需要继续留意和完善的生命周期点。

---

## 9. 一句话总结

当前 DDS 层已经被收敛成一个更真实的结构：

> **runtime 创建 Fast DDS 实体，translation 负责类型转换，ingress 把命令送进 core，egress 把状态发回 DDS，factory 负责把这些对象装起来。**

它不再试图发明一层额外的 transport 小框架，而是直接围绕当前代码真正需要的 DDS ↔ core 数据流来组织。
