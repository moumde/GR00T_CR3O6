# 本次联调接入信息

| 项目 | 本次约定 |
| --- | --- |
| HTTPS 域名 | https://webpage-talented-employees-usd.trycloudflare.com |
| 公网端口 | TCP 443 |
| 协议 | HTTPS/JSON，使用配套 Python SDK；不是原生 ZMQ 接口 |
| 失效时间 | 以新服务返回的 `expires_utc` 为准；到期停止，不自动续期 |
| 鉴权 | Bearer Token，私有凭据通过约定安全渠道单独提供，不在此包内 |
| 实际配置 | 随包 session-metadata.json；可用 SDK 的 get_modality_config() 再次获取 |
| 网络要求 | 无需 VPN，允许该域名 DNS 解析及出站 TCP 443；企业白名单要求需确认 |
| 使用范围 | 影子预测与对比，不下发机器人命令，不作为硬实时伺服接口 |

设置 `CUSTOMER_API_URL` 为上述域名，`CUSTOMER_API_TOKEN_FILE` 为私有凭据文件路径，按配套说明调用。网络请求不得阻塞本地控制线程；连续动作更新、队列衔接和断连停止方式须根据客户控制器另行确认。

临时通道由 Cloudflare 转发，TLS 在其边缘终止，图像与状态经过该第三方处理。仅上传已获授权的联调数据。临时通道不提供生产可用性保证；不要在共享文档、截图或日志中放入密钥和原始观测。
