# 机械臂与灵巧手推理服务接入说明

本服务接收三路相机图像和设备实测状态，返回动作预测序列。接入范围为观测上传与结果接收，供记录、对比和影子联调使用；服务及 SDK 不向机器人下发执行命令。

## 接入信息

| 项目 | 接入约定 |
| --- | --- |
| 服务域名 | 使用对接联系人为本次会话单独提供的 HTTPS 域名，不直接连接 IP。本说明不包含访问地址或密钥。 |
| 公网端口 | TCP 443。 |
| 通信协议 | HTTPS/JSON，使用配套 Python SDK；不是原生 ZMQ 接口。 |
| 实际接口配置 | 通过 SDK 的 `get_modality_config()` 获取本次会话配置，返回 JSON 字典；不是原生 RPC 配置对象。 |
| 网络与鉴权 | 公网 HTTPS + Bearer Token，无需 VPN。客户网络须允许该域名的 DNS 解析及出站 TCP 443；企业域名/IP 白名单要求需提前确认。 |

开始联调前，请确认已收到本次会话的**服务域名、访问凭据、失效时间及经核对的实际配置回包**。下文参数为接口约定，不是某次会话的实测回包。域名可能随会话变更，请勿使用其他连通性检查地址替代推理地址。

会话有效期以本次单独签发的失效时间为准，到期停止且不自动续期。健康接口成功只说明服务可访问，不等于推理请求已经成功。未确认本次访问资料前，请勿上传业务观测。

## 客户端安装

使用 Python 3.11 或更高版本，在配套 SDK 包目录中安装依赖：

```bash
python -m pip install -r configs/customer_client_requirements.txt
```

设置两个环境变量：

| 变量 | 内容 |
| --- | --- |
| `CUSTOMER_API_URL` | 本次会话的 HTTPS 域名地址，不带接口路径 |
| `CUSTOMER_API_TOKEN_FILE` | 通过约定安全渠道取得的私有凭据文件路径 |

不要把密钥正文写入 URL、命令参数、共享文档或截图。凭据文件应放在非同步目录，并限制为本人可读；Linux/macOS 使用 `chmod 600`。请保留 SDK 及第三方依赖依法随附的许可声明。

## 调用示例

以下零值数组仅用于说明数据结构，实际联调须替换为同步采集的 RGB 图像与设备实测状态，不可作为真实设备控制输入。

```python
import os
import numpy as np
from scripts.customer_client import CustomerPolicyClient

images = {
    "top": np.zeros((224, 224, 3), dtype=np.uint8),
    "wrist": np.zeros((224, 224, 3), dtype=np.uint8),
    "side": np.zeros((224, 224, 3), dtype=np.uint8),
}
state = np.zeros(12, dtype=np.float32)

with CustomerPolicyClient(
    os.environ["CUSTOMER_API_URL"],
    os.environ["CUSTOMER_API_TOKEN_FILE"],
    timeout=10,
) as client:
    configuration = client.get_modality_config()
    assert configuration["ready"] is True
    client.reset()
    actions, info = client.infer(images, state)
    assert actions.shape == (16, 12)
    assert info["mode"] == "shadow"
```

SDK 自动处理无损图像编码、请求标识和协议检查。若相机输出为 BGR，需显式转换为 RGB；SDK 不猜测颜色顺序，也不自动缩放图像。保持三路相机与状态的时间同步，不排队发送过期观测。

复用同一个客户端连接，并在同一个同步工作线程中完成创建、调用及关闭。异步应用请将完整客户端生命周期放在专用同步工作线程中。独立任务轮次之间调用 `reset()`。

网络请求不得放在实时控制线程中同步等待。动作更新周期、队列衔接及断连停止方式须由双方根据客户控制器接口另行确认；当前接入不构成连续真机控制验收。

## 输入与输出

| 项目 | 约定 |
| --- | --- |
| 相机名称 | 严格为 `top`、`wrist`、`side` 三路 |
| 单张图像 | RGB、`uint8`、形状 `(224, 224, 3)` |
| 图像传输 | 单帧 RGB PNG + base64，由 SDK 处理 |
| 状态输入 | 12 个有限数值，兼容 `float32`，形状 `(12,)` |
| 动作预测 | 形状 `(16, 12)`，单位见下表 |
| 序列步间隔 | `dt_seconds: 0.05`；不代表网络时延或实际控制频率保证 |
| 固定通道 | 从零计数的第 9、10、11 通道保持为零 |
| 请求大小 | HTTP 请求体不超过 2 MiB，单张 PNG 不超过 512 KiB；不接受压缩的 HTTP 请求体 |
| 使用方式 | `mode: shadow`、`robot_execution_enabled: false` |

| 通道索引（从零计数） | 状态含义 | 动作含义 |
| --- | --- | --- |
| 0-5 | 机械臂第 1 至第 6 关节的实测位置，弧度 | 对应关节的绝对目标位置，弧度；不是增量 |
| 6 | `thumb_flex`，设备原始位置值 | 对应设备原始指令值 |
| 7 | `thumb_yaw`，设备原始位置值 | 对应设备原始指令值 |
| 8 | `index_flex`，设备原始位置值 | 对应设备原始指令值 |
| 9-11 | `middle_flex`、`ring_flex`、`little_flex`，固定为零 | 对应通道固定为零 |

不要将关节值转为角度，不要对输入输出再做归一化，也不要把手部原始数值转成 [0,1]。任务约定为“拾取电机，将其放到右侧，凸出的一侧朝左”；SDK 默认填入对应任务描述。其他任务须另行确认，不接受任意任务指令。

## 配置与接口

`get_modality_config()` 返回本次服务配置。请核对 `ready`、`expires_utc`、`cameras`、`image_shape`、`state_names`、`action_names`、`action_shape`、`fixed_zero_channels`、`dt_seconds`。配置与设备映射不一致时，停止发送观测并联系对接负责人。

| 方法与路径 | 鉴权 | 用途 |
| --- | --- | --- |
| `GET /healthz` | 无 | 查询服务存活状态和会话失效时间 |
| `GET /v1/metadata` | Bearer Token | 获取接口配置，对应 `get_modality_config()` |
| `POST /v1/infer` | Bearer Token | 提交一次观测并取得一段预测 |
| `POST /v1/reset` | Bearer Token | 在独立任务轮次之间重置 |

认证头为 `Authorization: Bearer <private token>`，正文使用 UTF-8 `application/json`。SDK 负责封装所需字段。直接实现 HTTP 客户端时须遵循同一接口约定：

- 推理请求：`contract_version`、`request_id`、`images`、`state`，以及可省略的 `task`。`contract_version` 为中性的接口协议标识 `customer-api-v1`。
- `request_id`：1-64 个 ASCII 字母、数字、下划线或连字符。每次请求使用新标识。
- 成功响应：同一 `request_id`、`contract_version`、`actions`、`dt_seconds`、`server_elapsed_seconds`、`mode`、`robot_execution_enabled`。
- 重置请求：仅含 `contract_version` 和 `request_id`；响应包含 `reset: true` 及请求标识、协议、耗时和使用方式字段。
- 错误响应：`error.code`、`error.message`，以及可取得的 `request_id`，不包含动作数组。

不要在查询参数中传密钥，不要关闭 TLS 证书校验，不接受跳转地址。每次只允许一个请求在途，包括配置查询和重置。SDK 默认总超时为十秒，包含图像编码，不自动重试推理请求。

## 异常处理

| 结果 | 处理方式 |
| --- | --- |
| HTTP 400 / 415 | 检查字段、编码、尺寸、固定通道及 Content-Type |
| HTTP 401 | 核对本次会话的访问凭据 |
| HTTP 413 | 将图像和请求体限制在约定大小内 |
| HTTP 429 | 已有请求在处理；不要堆积旧观测 |
| HTTP 502 | 不使用该结果，也不回退执行之前的预测；联系对接负责人 |
| HTTP 503 | 停止调用，确认会话可用性和失效时间 |
| HTTP 504 / SDK 超时 | 丢弃失败请求；后续重新发起时须使用新的同步观测 |

不要执行迟到的结果、复用失败前的动作段，或自动重试旧观测。预测结果不构成真机执行授权，也不替代设备侧的限位、急停、互锁和人工接管措施。

## 数据与支持

仅传输已获授权用于本次联调的数据。临时接入通道使用 Cloudflare 转发 HTTPS 请求，TLS 在其边缘终止，图像和状态会经过该第三方处理。