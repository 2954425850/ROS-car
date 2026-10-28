#!/usr/bin/env node
/**
 * ============================================================================
 * WebSocket 验收探针（验收 D1~D4、C1 观测、G8、C9 的 WS 路径）
 * ============================================================================
 *
 * 为什么自己实现 WS 帧编解码：服务器上只有 JRE（没有 Node、没有 python ws 库），
 * 而本机前端工程也没装 `ws` 依赖。与其引入一个新依赖，不如用 Node 内置 http 做
 * 升级握手 + 手写最小的帧编解码 —— 协议本身只有几十行。
 * 这样验收跑在本机、打到「隧道转发过来的后端」上，得到的是**真实链路的原始输出**。
 *
 * 用法：
 *   node deploy/tools/ws-probe.mjs --url ws://127.0.0.1:18080/ws/robot --token <jwt> \
 *        --status-seconds 30 --expect-status-hz 1
 *
 * 输出：每一步的原始结果（握手状态码、收到的消息原文、计数），退出码 0/1。
 * ============================================================================
 */

import http from 'node:http'
import https from 'node:https'
import crypto from 'node:crypto'
import { URL } from 'node:url'

const args = process.argv.slice(2)
function arg(name, def) {
  const i = args.indexOf(`--${name}`)
  return i >= 0 && args[i + 1] ? args[i + 1] : def
}

const WS_URL = arg('url', 'ws://127.0.0.1:18080/ws/robot')
const TOKEN = arg('token', '')
const STATUS_SECONDS = Number(arg('status-seconds', '32'))
const NO_TOKEN = args.includes('--no-token')
const PING = !args.includes('--no-ping')
const CONTROL = arg('control', '')          // 例如 '{"type":"control","device":"drive","vx":0.2,"wz":0}'
const REST_KEYS = arg('rest-keys', '')      // REST /api/robot/status 的 key 列表（逗号分隔），用于 D3 比对
const HOST_HEADER = arg('host', '')         // 走 Nginx 时用来指定 Host / SNI

const results = []
function check(id, ok, detail) {
  results.push({ id, ok, detail })
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${id}  ${detail}`)
}

/* ---------------------------------------------------------------- 帧编解码 */

function encodeFrame(payload, opcode = 0x1) {
  const data = Buffer.from(payload, 'utf8')
  const mask = crypto.randomBytes(4)
  let header
  if (data.length < 126) {
    header = Buffer.alloc(2)
    header[1] = 0x80 | data.length
  } else if (data.length < 65536) {
    header = Buffer.alloc(4)
    header[1] = 0x80 | 126
    header.writeUInt16BE(data.length, 2)
  } else {
    header = Buffer.alloc(10)
    header[1] = 0x80 | 127
    header.writeBigUInt64BE(BigInt(data.length), 2)
  }
  header[0] = 0x80 | opcode
  const masked = Buffer.alloc(data.length)
  for (let i = 0; i < data.length; i++) masked[i] = data[i] ^ mask[i % 4]
  return Buffer.concat([header, mask, masked])
}

/** 从累积缓冲区里切出完整帧；返回 [{opcode, payload}] 与剩余缓冲。 */
function decodeFrames(buf) {
  const frames = []
  let offset = 0
  while (offset + 2 <= buf.length) {
    const b0 = buf[offset]
    const b1 = buf[offset + 1]
    const opcode = b0 & 0x0f
    const masked = (b1 & 0x80) !== 0
    let len = b1 & 0x7f
    let p = offset + 2
    if (len === 126) {
      if (p + 2 > buf.length) break
      len = buf.readUInt16BE(p); p += 2
    } else if (len === 127) {
      if (p + 8 > buf.length) break
      len = Number(buf.readBigUInt64BE(p)); p += 8
    }
    let maskKey = null
    if (masked) {
      if (p + 4 > buf.length) break
      maskKey = buf.subarray(p, p + 4); p += 4
    }
    if (p + len > buf.length) break
    const raw = Buffer.from(buf.subarray(p, p + len))
    if (maskKey) for (let i = 0; i < raw.length; i++) raw[i] ^= maskKey[i % 4]
    frames.push({ opcode, payload: raw.toString('utf8') })
    offset = p + len
  }
  return { frames, rest: buf.subarray(offset) }
}

/* ---------------------------------------------------------------- 握手 */

function connect({ withToken }) {
  return new Promise((resolve, reject) => {
    const u = new URL(WS_URL)
    const secure = u.protocol === 'wss:'
    const query = withToken ? `?token=${encodeURIComponent(TOKEN)}` : ''
    const key = crypto.randomBytes(16).toString('base64')
    const headers = {
      Connection: 'Upgrade',
      Upgrade: 'websocket',
      'Sec-WebSocket-Key': key,
      'Sec-WebSocket-Version': 13,
    }
    if (HOST_HEADER) headers.Host = HOST_HEADER

    const opts = {
      hostname: u.hostname,
      port: u.port || (secure ? 443 : 80),
      path: u.pathname + query,
      headers,
    }
    if (secure) {
      // 自签证书：IP 直访没有可信 CA，只能放行（这正是 J2/J5 用 curl -k 的原因）
      opts.rejectUnauthorized = false
      if (HOST_HEADER) opts.servername = HOST_HEADER
    }

    const mod = secure ? https : http
    const req = mod.request(opts)
    req.on('upgrade', (res, socket, head) => {
      resolve({ ok: true, status: res.statusCode, socket, head })
    })
    req.on('response', (res) => {
      let body = ''
      res.on('data', (c) => { body += c })
      res.on('end', () => resolve({ ok: false, status: res.statusCode, body }))
    })
    req.on('error', reject)
    req.end()
  })
}

/* ---------------------------------------------------------------- 主流程 */

const messages = []
let socket = null

async function main() {
  console.log(`目标: ${WS_URL}${NO_TOKEN ? '（故意不带 token）' : '（带 token）'}`)
  console.log('')

  const r = await connect({ withToken: !NO_TOKEN })

  if (NO_TOKEN) {
    check('D1', r.status === 401, `不带 ?token= 握手 → HTTP ${r.status}（期望 401）`)
    if (r.body) console.log(`      响应体: ${r.body.trim()}`)
    return
  }

  if (!r.ok) {
    check('D1', false, `带 token 握手失败 → HTTP ${r.status} ${r.body ?? ''}`)
    return
  }
  check('D1', r.status === 101, `带合法 token 握手 → HTTP ${r.status} Upgrade（期望 101）`)

  socket = r.socket
  let buf = r.head && r.head.length ? Buffer.from(r.head) : Buffer.alloc(0)

  socket.on('data', (chunk) => {
    buf = Buffer.concat([buf, chunk])
    const { frames, rest } = decodeFrames(buf)
    buf = rest
    for (const f of frames) {
      if (f.opcode === 0x1) {
        try {
          messages.push(JSON.parse(f.payload))
        } catch {
          messages.push({ type: '(非 JSON)', raw: f.payload })
        }
      } else if (f.opcode === 0x8) {
        console.log('      服务端关闭连接')
      } else if (f.opcode === 0x9) {
        socket.write(encodeFrame(f.payload, 0xA))   // pong
      }
    }
  })

  // ---- D4：ping → pong ----
  if (PING) {
    const before = messages.length
    socket.write(encodeFrame(JSON.stringify({ type: 'ping' })))
    await sleep(1200)
    const pong = messages.slice(before).find((m) => m.type === 'pong')
    check('D4', !!pong, pong
      ? `发 {"type":"ping"} → 收到 {"type":"pong","ts":${pong.ts}}`
      : '发 ping 后 1.2s 内没收到 pong')
  }

  // ---- C1：连接后立刻应收到一条 status ----
  const firstStatus = messages.find((m) => m.type === 'status')
  check('D-on-connect', !!firstStatus,
    firstStatus ? '连接后立即收到一条 status（不必等 1s 周期）' : '连接后没有立即收到 status')

  // ---- 可选的 control 注入（C2/C9 的 WS 路径）----
  if (CONTROL) {
    const before = messages.length
    socket.write(encodeFrame(CONTROL))
    await sleep(1000)
    const err = messages.slice(before).find((m) => m.type === 'event' && m.event === 'error')
    console.log(`      注入 ${CONTROL} → ${err ? '错误事件 ' + JSON.stringify(err.payload) : '已接受（无错误事件）'}`)
  }

  // ---- D2：status 推送频率 ----
  messages.length = 0
  const t0 = Date.now()
  await sleep(STATUS_SECONDS * 1000)
  const elapsed = (Date.now() - t0) / 1000
  const statuses = messages.filter((m) => m.type === 'status')
  const hz = statuses.length / elapsed
  const expect = STATUS_SECONDS   // 1 Hz ⇒ 每秒约 1 条
  const okHz = Math.abs(hz - 1) <= 0.15
  check('D2', okHz && statuses.length > 0,
    `${elapsed.toFixed(1)}s 内收到 ${statuses.length} 条 type:"status" → ${hz.toFixed(2)} Hz（期望 ~1 Hz）`)

  // ---- D3：WS status 与 REST status 字段一致 ----
  if (REST_KEYS && statuses.length) {
    const wsKeys = Object.keys(statuses[statuses.length - 1].payload).sort()
    const restKeys = REST_KEYS.split(',').map((s) => s.trim()).sort()
    const same = wsKeys.length === restKeys.length && wsKeys.every((k, i) => k === restKeys[i])
    check('D3', same, same
      ? `WS 与 REST 的 status key 集合完全一致：${wsKeys.join(',')}`
      : `不一致\n      WS  : ${wsKeys.join(',')}\n      REST: ${restKeys.join(',')}`)
  }

  // ---- D5/D6/D7：MOCK 标注 / voltage null / gimbal.source ----
  if (statuses.length) {
    const s = statuses[statuses.length - 1].payload
    check('D5', s.gateway === 'MOCK' || s.gateway === 'LIVE', `status.gateway = ${s.gateway}`)
    check('D6', s.voltage === null, `status.voltage = ${JSON.stringify(s.voltage)}（期望 null = 不可用）`)
    check('D7', s.gimbal && s.gimbal.source === 'command',
      `status.gimbal = ${JSON.stringify(s.gimbal)}（source 恒为 "command" = 指令值）`)
    console.log(`      最后一条 status 原文: ${JSON.stringify(statuses[statuses.length - 1])}`)
  }

  // ---- G8：aiobjs（若 8556 有上游才会有）----
  const ai = messages.filter((m) => m.type === 'aiobjs')
  console.log(`      （可选）type:"aiobjs" 共 ${ai.length} 条${ai.length ? '，示例: ' + JSON.stringify(ai[ai.length - 1]).slice(0, 200) : ''}`)

  // ---- event 类消息（故障 / 错误）----
  const events = messages.filter((m) => m.type === 'event')
  if (events.length) console.log(`      收到 event: ${JSON.stringify(events.slice(0, 3))}`)

  socket.end()
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)) }

main()
  .then(() => {
    const failed = results.filter((r) => !r.ok)
    console.log(`\n探针完成：${results.length} 项，失败 ${failed.length} 项`)
    process.exit(failed.length ? 1 : 0)
  })
  .catch((e) => {
    console.error('探针异常:', e)
    process.exit(2)
  })
