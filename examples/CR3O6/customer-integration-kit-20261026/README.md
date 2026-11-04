# 预测接口 v2 接入与迁移说明（2026-10-26 会话）

**本次预测接口已上线并通过真实观测请求验收，固定到期为北京时间 2026-10-29 11:15:26。** 使用本次配套 SDK、会话信息和实际配置回包；旧客户端不兼容，不能只改地址后直接运行设备。

服务接收三路图像和实测状态，返回动作预测。服务及 SDK 不向设备下发命令；预测接口可用不代表现场执行程序、手指物理映射、连续控制或抓取效果已经验收。

**性能限制：本次公网高清请求记录耗时约 1.10–4.50 秒，超过约 0.8 秒的名义动作段时长。本次仅通过预测接口验收，未通过连续实时执行或抓取成功验收。不能通过放宽动作年龄上限、重放旧预测或放慢动作来掩盖这一缺口。**

## 1. 连接与有效期

| 项目 | v2 约定 |
| --- | --- |
| HTTPS 域名 | `https://festival-usd-payday-surveys.trycloudflare.com`；旧会话域名不再作为接入地址 |
| 公网端口与协议 | TCP 443，HTTPS/JSON；使用域名，不直接连接 IP |
| 鉴权 | 复用既有客户 API Token，SDK 从客户本机的私有凭据文件读取并附加 Bearer 认证头 |
| 接口版本 | `customer-api-v2`，业务路径为 `/v2`；不是旧 SDK／旧请求格式的兼容升级 |
| 网络条件 | 允许新域名 DNS 解析及出站 TCP 443；企业代理和白名单须按新域名核对。SDK 不自动读取环境代理配置 |
| 实际启动 | 北京时间 2026-10-26 11:15:26（UTC+08:00，显示到秒）；精确 UTC 为 `2026-10-26T03:15:26.704984+00:00` |
| 固定到期 | 北京时间 2026-10-29 11:15:26（UTC+08:00，显示到秒）；`expires_utc` 为 `2026-10-29T03:15:26.704984+00:00` |
| 会话期限 | 自上述实际启动起 72 小时，精确 259,200 秒，到期停止、不自动续期。领取材料、更新地址或首次调用都不重新计时 |

本次**更换域名、复用既有 Token**，但不能只改地址：仍须更新 SDK 和下述输入输出处理。原会话已结束，本次是独立新会话，不是续期。更换客户程序的接入地址前须完成第 6 节的现场维护确认。

原客户 Token 已通过本次公网配置查询核对：鉴权请求返回 HTTP 200、`ready=true`，未鉴权请求返回 HTTP 401，旧 `/v1/metadata` 返回 HTTP 404。随包 `public-metadata.json` 是本次实际 `/v2/metadata` 响应快照，不是示例 schema；快照不保证后续持续可用，使用时仍须由 SDK 查询并校验。

## 2. 客户端必须调整的内容

| 项目 | v2 接入要求 |
| --- | --- |
| 接入地址 | 将 `CUSTOMER_API_URL` 更新为第 1 节的新 HTTPS 域名，并核对网络白名单；既有客户 Token 及其私有文件不变 |
| SDK 与路由 | 使用配套 v2 SDK 和 `/v2`；不得只修改旧程序中的版本字符串。新版不提供 `/v1` 兼容接口 |
| 图像 | 三路原始 RGB，宽 640、高 480；移除上传前的 224 方图缩放、黑边补齐或裁剪。不要将旧小图放大冒充原始图像 |
| 中指反馈与目标 | 上传真实 `state[9]`，保留 `actions[:, 9]`；移除将中指与后两指一起清零、截断或按旧规则拒绝的逻辑 |
| 固定通道 | 只有索引 10、11 固定为零；若实测不符合约定，停止并核对映射，不伪造反馈来通过校验 |
| 抓法 | 默认三指，也允许显式两指；使用配套 SDK 的两条规范任务文本，不沿用旧任务字符串或任意改写 |
| 返回字段 | 按下表检查职责与范围；不再读取旧的 `mode`、`robot_execution_enabled` 字段决定 v2 行为 |
| 本地执行 | 设备许可、队列、限位、异常停止和人工接管仍由现场程序负责；不能因为远端字段变化而移除现场保护 |

元数据、成功推理和重置响应均包含以下字段：

| 字段 | 固定值 | 含义 |
| --- | --- | --- |
| `service_role` | `action_prediction` | 服务提供动作预测 |
| `execution_owner` | `customer_controller` | 设备执行由客户控制程序负责 |
| `integration_scope` | `prediction_integration` | 本次范围是预测接口集成 |
| `hardware_enable_state` | `not_reported` | 服务未获得可核实的现场使能状态；既不代表已关闭，也不代表已开启或获准运动 |

以上字段表用于解释约定，实际配置快照见随包 `public-metadata.json`。`ready=true` 也只说明预测服务就绪，不是现场安全联锁。

## 3. 图像、状态、动作与任务

| 项目 | 要求 |
| --- | --- |
| 图像字典 | 恰好为 `top`、`wrist`、`side` 三路；保持相机角色对应，不互换 |
| 单张图像 | RGB、`uint8`，数组形状 `(480, 640, 3)`，即高、宽、通道 |
| 图像传输 | 单帧 RGB PNG + base64，SDK 负责无损编码；不接受 JPEG、灰度图或带透明通道的 PNG |
| 图像处理 | 客户端上传原始尺寸；服务内部按最短边 384、保留完整视野处理，不在客户端预先缩到 384 或加黑边 |
| 状态 | `(12,)`，12 个可表示为 `float32` 的有限数值；使用设备实测值，不用上一条目标替代 |
| 动作 | `(16, 12)`，每步同时包含臂和手的目标；不是已经发送到设备的轨迹 |
| 时间间隔 | `dt_seconds=0.05`，是预测目标间隔，不是网络延迟或现场 20Hz 连续控制保证 |
| 大小限制 | 单张 PNG 不超过 1 MiB（1,048,576 字节）；完整 HTTP 请求体含 base64 和 JSON 不超过 5 MiB（5,242,880 字节）。不接受压缩的 HTTP 请求体 |

| 索引（从零计数） | 状态 | 动作 |
| --- | --- | --- |
| 0–5 | 六个机械臂关节实测位置，rad | 六个关节的绝对目标位置，rad；不是增量、速度或末端位姿 |
| 6 | `thumb_flex` 原始位置值 | 对应原始指令值 |
| 7 | `thumb_yaw` 原始位置值 | 对应原始指令值 |
| 8 | `index_flex` 原始位置值 | 对应原始指令值 |
| 9 | `middle_flex` 真实原始位置值，可变 | 对应原始指令值，可变，不强制清零 |
| 10、11 | `ring_flex`、`little_flex`，固定零 | 对应通道固定零 |

臂、手都不在客户端额外归一化。若设备接口使用角度，只在设备适配层完成一次 rad／degree 转换；手部 raw 不是弧度或 `[0,1]`，raw 零值也不能直接解释为“张开”。关节顺序、正方向、零位和手部物理通道仍须现场确认。

### 规范任务

推荐用 `TASKS["three_finger"]` 或 `TASKS["two_finger"]` 获取文本。`three_finger`／`two_finger` 是 SDK 字典键，**不是 HTTP `task` 接受的简写**；HTTP 传入的是以下完整文本。

**三指（默认）**

> Use the thumb, index finger, and middle finger to pick up the motor and place it on the right side with the protruding side facing left. Keep the ring and little fingers at their recorded zero positions.

**两指（显式选择）**

> Use the thumb and index finger to pick up the motor and place it on the right side with the protruding side facing left. Keep the middle, ring, and little fingers at their recorded zero positions.

省略 `task` 时使用三指文本；不接受任意任务、中文替代文本或自行改写。响应 `task` 回显本次采用的完整文本，SDK 会核对一致性。

选择两指时仍上传真实中指反馈，不能据任务名称将 `state[9]` 或返回目标硬改为零。任务回显只证明本次采用了哪条指令，不证明现场已切换抓法；接口也不提供运动中的安全抓法切换保证。

## 4. SDK 最小接入

使用 Python 3.11 或更高版本，保留正式 SDK 包的目录结构，在包根目录安装依赖：

```powershell
python -m pip install -r .\configs\customer_client_requirements.txt
```

将 `CUSTOMER_API_URL` 设为 `https://festival-usd-payday-surveys.trycloudflare.com`，不带 `/v2`；保留原 `CUSTOMER_API_TOKEN_FILE` 设置及既有 Token。后者是本机凭据文件的完整路径，不是目录或令牌正文；无需更换 Token 文件。凭据放在非同步目录并限制为本人可读；Linux/macOS 使用 `chmod 600`。不要将 Token 写入 URL、命令参数、源码、共享文档或截图。

以下函数仅获取一个独立轮次的预测，**不包含设备调用**。由调用方传入真实 RGB 图像字典和实测状态；不要用虚构的零值反馈替代现场观测。

```python
import os
from scripts.customer_client import CustomerPolicyClient, TASKS


def predict_one_round(images, state, *, grasp="three_finger"):
    task = TASKS[grasp]
    with CustomerPolicyClient(
        os.environ["CUSTOMER_API_URL"],
        os.environ["CUSTOMER_API_TOKEN_FILE"],
        timeout=10,
    ) as client:
        configuration = client.get_modality_config()
        client.reset()
        actions, info = client.infer(images, state, task=task)
    return actions, info, configuration
```

- `client.infer(images, state)` 不传 `task` 也默认三指；显式两指用 `task=TASKS["two_finger"]`。SDK 没有单独的 `grasp` 请求参数。
- SDK 核对实际配置、请求标识、任务回显、职责字段、动作形状、固定通道和步间隔；不符则报错，不把异常响应当动作。
- 连续采集时复用同一个客户端，在同一同步工作线程中创建、调用并关闭；不要每帧重置。异步应用使用专用同步工作线程，实时控制线程不等待 HTTP。
- 相机为 BGR 时先明确转换为 RGB；SDK 不猜颜色顺序或自动改尺寸。保存本地采集时刻并核对图像／状态时序，不堆积旧观测。
- `reset()` 仅重置预测侧状态，不是设备回零、取消控制器队列、现场急停或恢复运动许可。

## 5. HTTP 字段与异常

| 方法与路径 | 鉴权 | 用途 |
| --- | --- | --- |
| `GET /healthz` | 无 | 仅存活检查，不验证预测可用性 |
| `GET /v2/metadata` | Bearer | 实际配置，对应 `get_modality_config()` |
| `POST /v2/infer` | Bearer | 一次观测对应一段预测 |
| `POST /v2/reset` | Bearer | 重置预测侧状态 |

认证头为 `Authorization: Bearer <API Token>`。请求正文使用 UTF-8 `application/json`，不用查询参数传数据或凭据，不关闭 TLS 校验，不接受跳转地址。

- **推理请求**：`contract_version`、`request_id`、`images`、`state`，以及可省略的 `task`；不添加其他字段。
- **请求标识**：`request_id` 为 1–64 个 ASCII 字母、数字、下划线或连字符，每次使用新值。
- **推理成功**：同一 `request_id`、`contract_version`、`actions`、`dt_seconds`、`task`、`server_elapsed_seconds`，以及第 2 节的四个职责字段。
- **重置**：请求仅含 `contract_version` 和 `request_id`；成功响应为同一标识、协议、`reset=true`、耗时和四个职责字段。
- **实际配置**：核对 `contract_version`、`ready`、`expires_utc`、四个职责字段、`cameras`、`image_shape`、`image_encoding`、`state_names`、`action_names`、`action_shape`、`fixed_zero_channels`、`arm_units`、`hand_units`、`dt_seconds`、默认 `task`、`supported_tasks`、`max_request_bytes`、`max_image_bytes`。
- **错误**：`error.code`、`error.message` 及可取得的 `request_id`，没有动作数组。`server_elapsed_seconds` 不包含完整采集和公网往返时间，不能充当端到端时延。

每次只允许一个请求在途，包括配置查询和重置。SDK 默认总超时 10 秒，推理时包含图像编码；不自动重试。这个超时值**不是机器人可以等待的时限**。

| 结果 | 处理 |
| --- | --- |
| HTTP 400／415 | 核对版本、字段、完整任务文本、RGB PNG、尺寸、真实固定通道和 Content-Type |
| HTTP 401 | 核对既有凭据文件及认证头，不在聊天或日志中展开 Token |
| HTTP 404／405、SDK `incompatible_service` | 核对发布状态、接口路径、SDK 版本和实际配置；不尝试自动回退到旧接口 |
| HTTP 413 | 检查单张 PNG 与完整请求体上限，不通过裁剪／降清晰度静默绕过输入约定 |
| HTTP 429 | 已有请求在途，不并发补发或积压旧观测 |
| HTTP 408／504、网络或 SDK 超时 | 本次无可用结果，不重试旧观测；恢复后需新观测和新请求标识 |
| HTTP 502／503、SDK 响应校验失败 | 不使用结果或复用失败前动作；核对服务状态／会话到期并联系对接负责人 |

16 步按约定间隔名义覆盖约 800ms，但不是“返回后还能完整执行 800ms”。采集、传输与计算已消耗时间；本地必须另行验证动作年龄、连续性、队列和停止语义，不能从收到响应时给整段重新计时，也不能把网络恢复等同恢复运动。

### 本次时延应如何理解

本次公网高清请求记录耗时约 **1.10–4.50 秒**，是接口验收环境下的有限观测，包含请求编码、传输、服务处理及客户端校验，不含客户现场相机／状态采集。它不代表客户现场网络表现，也不据此给出 P95、时延 SLA 或实时控制承诺。

上述耗时已超过约 **0.8 秒**的名义动作段覆盖时间，不能作为持续衔接动作的依据。当前仅用于预测接口集成与不下发设备的结果核对；连续执行需要另行解决并验收时延、动作时间对齐和本地控制条件，不能靠调大允许动作年龄或 SDK 超时来补足。

## 6. 更换接入地址前确认与发布后核对

**更换客户程序的接入地址前，必须由客户授权人员按现场安全流程停止旧执行程序，并明确确认其已停止、现场维护条件已满足。** 旧程序在网络错误等异常路径可能触发回零，不能用直接断网或停止预测服务代替现场停机。旧会话到期或新地址可访问都不能证明现场已经停止；服务端无法代为判断或确认硬件状态。

本次联调包仅含五个接入文件：

| 包内路径 | 内容 |
| --- | --- |
| `README.md` | 本接入与迁移说明 |
| `scripts/customer_client.py` | 配套 v2 SDK |
| `configs/customer_client_requirements.txt` | 客户端依赖清单 |
| `public-metadata.json` | 已核对的实际配置响应快照 |
| `session-info.md` | 本次地址、精确期限、认证方式和使用限制 |

包内不含访问凭据或设备执行程序。此次验收包含真实观测的预测请求，不只是健康或配置检查；通过结论仍仅限预测接口，不是现场执行或抓取验收。

客户在更新客户端后，先做不下发设备命令的观测／预测核对。现场执行仍须单独确认通道映射、时间语义、停止与接管流程；不能通过删除旧检查、伪造职责字段或强制清零中指来宣布兼容。

## 7. 数据处理与许可

仅上传已获授权用于本次临时联调的数据。HTTPS 通道由 Cloudflare 转发，TLS 在其边缘终止，图像和状态会经过该第三方处理；这不是纯内网直连或通用生产服务承诺。

配套 SDK 和第三方依赖依法随附的许可声明应保留。故障沟通提供请求标识和错误码，不附 Token、原始观测或完整现场日志，除非已确认相应数据权限与安全传递方式。
