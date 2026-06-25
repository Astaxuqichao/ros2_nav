#!/usr/bin/env python3
"""Small web UI for Navflex instruction service."""

import json
import os
import re
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List
from urllib.parse import urlparse

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from navflex_instruction_server.srv import ExecuteInstruction, GetCapabilities


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Navflex 指令控制台</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f7fb;
      --panel: #ffffff;
      --text: #172033;
      --muted: #667085;
      --line: #d9e2ef;
      --accent: #0f766e;
      --accent-2: #2563eb;
      --danger: #b42318;
      --ok: #067647;
      --warn: #b54708;
      --shadow: 0 10px 30px rgba(16, 24, 40, 0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.45;
    }
    main {
      width: min(1180px, calc(100vw - 32px));
      margin: 28px auto;
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(320px, 0.8fr);
      gap: 18px;
    }
    header {
      width: min(1180px, calc(100vw - 32px));
      margin: 28px auto 0;
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 16px;
    }
    h1 { margin: 0; font-size: 28px; letter-spacing: 0; }
    h2 { margin: 0 0 14px; font-size: 17px; letter-spacing: 0; }
    .subtitle { margin: 6px 0 0; color: var(--muted); font-size: 14px; }
    .status {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 10px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--panel);
      font-size: 13px;
      color: var(--muted);
      white-space: nowrap;
    }
    .dot { width: 8px; height: 8px; border-radius: 999px; background: var(--warn); }
    .dot.ok { background: var(--ok); }
    .dot.bad { background: var(--danger); }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
      padding: 18px;
    }
    .composer { display: grid; gap: 12px; }
    .pane + .pane {
      margin-top: 18px;
      padding-top: 18px;
      border-top: 1px solid var(--line);
    }
    textarea {
      width: 100%;
      min-height: 124px;
      resize: vertical;
      border: 1px solid #b8c5d8;
      border-radius: 6px;
      padding: 13px 14px;
      font: inherit;
      color: var(--text);
      outline: none;
      background: #fbfdff;
    }
    textarea:focus { border-color: var(--accent-2); box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.12); }
    .row { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
    button {
      border: 0;
      border-radius: 6px;
      padding: 10px 14px;
      font-weight: 650;
      font-size: 14px;
      color: #fff;
      background: var(--accent);
      cursor: pointer;
      min-height: 40px;
    }
    button.secondary { background: #344054; }
    button.ghost {
      color: var(--text);
      background: #eef3f8;
      border: 1px solid var(--line);
    }
    button:disabled { cursor: not-allowed; opacity: 0.58; }
    .examples { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 8px; }
    .chip {
      border: 1px solid var(--line);
      background: #f8fafd;
      border-radius: 999px;
      color: #344054;
      padding: 7px 10px;
      font-size: 13px;
      cursor: pointer;
    }
    .timeline { display: grid; gap: 10px; margin-top: 14px; }
    .event {
      display: grid;
      grid-template-columns: 92px 1fr;
      gap: 10px;
      padding: 11px 12px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfdff;
    }
    .event time { color: var(--muted); font-size: 12px; white-space: nowrap; }
    .event strong { display: block; font-size: 14px; margin-bottom: 2px; }
    .event p { margin: 0; color: var(--muted); font-size: 13px; overflow-wrap: anywhere; }
    .event.feedback { border-color: #b7d6ff; background: #f5f9ff; }
    .event.failed { border-color: #f4b6af; background: #fff8f7; }
    .event.completed { border-color: #a6e2c2; background: #f6fef9; }
    .chatlog {
      display: grid;
      gap: 8px;
      max-height: 300px;
      overflow: auto;
      padding: 2px;
    }
    .bubble {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px 12px;
      background: #fbfdff;
      font-size: 14px;
    }
    .bubble.user {
      justify-self: end;
      max-width: 86%;
      background: #eef8f6;
      border-color: #b9ded8;
    }
    .bubble.assistant {
      justify-self: start;
      max-width: 92%;
    }
    .bubble small {
      display: block;
      margin-top: 4px;
      color: var(--muted);
      font-size: 12px;
    }
    .result {
      border-left: 4px solid var(--line);
      padding: 12px 14px;
      background: #fbfdff;
      border-radius: 6px;
      min-height: 96px;
    }
    .result.ok { border-left-color: var(--ok); }
    .result.bad { border-left-color: var(--danger); }
    .kv { display: grid; grid-template-columns: 140px minmax(0, 1fr); gap: 8px 10px; margin-top: 8px; font-size: 13px; }
    .kv div:nth-child(odd) { color: var(--muted); }
    .steps {
      margin: 0;
      padding-left: 18px;
      color: var(--text);
    }
    .steps li + li { margin-top: 5px; }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      background: #101828;
      color: #e4e7ec;
      border-radius: 6px;
      padding: 12px;
      max-height: 260px;
      overflow: auto;
      font-size: 12px;
    }
    ul { margin: 0; padding-left: 18px; color: var(--muted); font-size: 13px; }
    li + li { margin-top: 7px; }
    @media (max-width: 860px) {
      header { align-items: start; flex-direction: column; }
      main { grid-template-columns: 1fr; }
      .event { grid-template-columns: 1fr; }
      .kv { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>Navflex 指令控制台</h1>
      <p class="subtitle">输入抽象语言指令，后端调用 Navflex 文本指令服务并返回导航过程与结果。</p>
    </div>
    <div class="row">
      <div class="status"><span id="statusDot" class="dot"></span><span id="statusText">连接中</span></div>
      <div class="status"><span id="llmDot" class="dot"></span><span id="llmText">LLM 检查中</span></div>
    </div>
  </header>

  <main>
    <section>
      <div class="pane">
        <h2>对话调用</h2>
        <div class="composer">
          <textarea id="chatInput" spellcheck="false" placeholder="例如：帮我去厨房、往前走半米、右转90度"></textarea>
          <div class="row">
            <button id="chatBtn">发送对话</button>
            <button id="clearChatBtn" class="ghost">清空对话</button>
          </div>
          <div class="chatlog" id="chatlog"></div>
        </div>
      </div>

      <div class="pane">
        <h2>直接指令</h2>
        <div class="composer">
          <textarea id="instruction" spellcheck="false" placeholder="例如：去 1.0 2.0、前进0.5米、左转90度、goto kitchen"></textarea>
          <div class="row">
            <button id="executeBtn">执行指令</button>
            <button id="capBtn" class="secondary">刷新能力</button>
            <button id="clearBtn" class="ghost">清空过程</button>
          </div>
        </div>
        <div class="examples" id="examples"></div>
      </div>

      <div class="pane">
        <h2>实时过程</h2>
        <div class="timeline" id="timeline"></div>
      </div>
    </section>

    <section>
      <h2>执行结果</h2>
      <div id="result" class="result">
        <strong>等待指令</strong>
        <div class="kv">
          <div>状态</div><div>尚未执行</div>
        </div>
      </div>
      <h2 style="margin-top:18px;">能力</h2>
      <ul id="capabilities"></ul>
      <h2 style="margin-top:18px;">原始响应</h2>
      <pre id="raw">{}</pre>
    </section>
  </main>

  <script>
    const instructionEl = document.getElementById('instruction');
    const chatInputEl = document.getElementById('chatInput');
    const executeBtn = document.getElementById('executeBtn');
    const chatBtn = document.getElementById('chatBtn');
    const capBtn = document.getElementById('capBtn');
    const clearBtn = document.getElementById('clearBtn');
    const clearChatBtn = document.getElementById('clearChatBtn');
    const timelineEl = document.getElementById('timeline');
    const chatlogEl = document.getElementById('chatlog');
    const resultEl = document.getElementById('result');
    const rawEl = document.getElementById('raw');
    const capabilitiesEl = document.getElementById('capabilities');
    const examplesEl = document.getElementById('examples');
    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    const llmDot = document.getElementById('llmDot');
    const llmText = document.getElementById('llmText');

    function now() {
      return new Date().toLocaleTimeString('zh-CN', { hour12: false });
    }

    let lastEventId = -1;

    function addEvent(title, detail, cls = '') {
      const item = document.createElement('div');
      item.className = `event ${cls}`.trim();
      item.innerHTML = `<time>${now()}</time><div><strong>${escapeHtml(title)}</strong><p>${escapeHtml(detail || '')}</p></div>`;
      timelineEl.prepend(item);
    }

    function addChat(role, text, meta) {
      const item = document.createElement('div');
      item.className = `bubble ${role}`;
      item.innerHTML = `${escapeHtml(text)}${meta ? `<small>${escapeHtml(meta)}</small>` : ''}`;
      chatlogEl.appendChild(item);
      chatlogEl.scrollTop = chatlogEl.scrollHeight;
    }

    function escapeHtml(value) {
      return String(value ?? '').replace(/[&<>"']/g, (char) => ({
        '&': '&amp;',
        '<': '&lt;',
        '>': '&gt;',
        '"': '&quot;',
        "'": '&#39;',
      }[char]));
    }

    function setStatus(ok, text) {
      statusDot.className = `dot ${ok ? 'ok' : 'bad'}`;
      statusText.textContent = text;
    }

    function setLlmStatus(data) {
      const ready = data.status === 'ready';
      const disabled = data.status === 'disabled';
      llmDot.className = `dot ${ready ? 'ok' : (disabled ? '' : 'bad')}`;
      const label = ready
        ? `LLM ${data.model}`
        : `LLM ${data.status || 'unknown'}`;
      llmText.textContent = label;
      if (data.reason) {
        addEvent('LLM 状态', data.reason, data.status === 'ready' ? '' : 'failed');
      }
    }

    function setResult(data) {
      resultEl.className = `result ${data.success ? 'ok' : 'bad'}`;
      const steps = Array.isArray(data.steps) ? data.steps : [];
      const stepsHtml = steps.length
        ? `<div>过程</div><div><ol class="steps">${steps.map((step) => `<li>${escapeHtml(step)}</li>`).join('')}</ol></div>`
        : '';
      resultEl.innerHTML = `
        <strong>${data.success ? '执行成功' : '执行失败'}</strong>
        <div class="kv">
          <div>类型</div><div>${escapeHtml(data.command_type || '-')}</div>
          <div>标准命令</div><div>${escapeHtml(data.normalized_command || '-')}</div>
          <div>消息</div><div>${escapeHtml(data.message || '-')}</div>
          <div>耗时</div><div>${formatDuration(data.elapsed_time)}</div>
          ${stepsHtml}
        </div>`;
      rawEl.textContent = JSON.stringify(data, null, 2);
    }

    function formatDuration(duration) {
      if (!duration) return '-';
      const sec = Number(duration.sec || 0) + Number(duration.nanosec || 0) / 1e9;
      return `${sec.toFixed(3)} s`;
    }

    async function api(path, body) {
      const response = await fetch(path, {
        method: body ? 'POST' : 'GET',
        headers: body ? { 'Content-Type': 'application/json' } : {},
        body: body ? JSON.stringify(body) : undefined,
      });
      const data = await response.json();
      if (!response.ok) {
        throw new Error(data.error || response.statusText);
      }
      return data;
    }

    async function loadCapabilities() {
      addEvent('刷新能力', '请求 /api/capabilities');
      const data = await api('/api/capabilities');
      setStatus(true, '已连接');
      capabilitiesEl.innerHTML = '';
      (data.capabilities || []).forEach((cap) => {
        const li = document.createElement('li');
        li.textContent = cap;
        capabilitiesEl.appendChild(li);
      });
      examplesEl.innerHTML = '';
      (data.examples || []).forEach((example) => {
        const chip = document.createElement('button');
        chip.className = 'chip';
        chip.textContent = example;
        chip.onclick = () => { instructionEl.value = example; instructionEl.focus(); };
        examplesEl.appendChild(chip);
      });
      addEvent('能力已更新', `${(data.capabilities || []).length} 项能力，${(data.examples || []).length} 个示例`);
    }

    async function loadLlmStatus() {
      const data = await api('/api/llm_status');
      setLlmStatus(data);
    }

    async function executeInstruction() {
      const instruction = instructionEl.value.trim();
      if (!instruction) {
        addEvent('输入为空', '请输入一条导航指令');
        instructionEl.focus();
        return;
      }
      executeBtn.disabled = true;
      addEvent('发送指令', instruction);
      try {
        addEvent('等待导航结果', '后端正在调用 Navflex instruction service');
        const data = await api('/api/execute', { instruction });
        setResult(data);
        setStatus(true, '已连接');
        addEvent(data.success ? '执行完成' : '执行失败', data.message || data.normalized_command || '无消息');
      } catch (err) {
        setStatus(false, '请求失败');
        const data = { success: false, command_type: 'web_error', normalized_command: '', message: String(err), elapsed_time: null };
        setResult(data);
        addEvent('请求失败', String(err));
      } finally {
        executeBtn.disabled = false;
      }
    }

    async function chat() {
      const message = chatInputEl.value.trim();
      if (!message) {
        chatInputEl.focus();
        return;
      }
      chatBtn.disabled = true;
      addChat('user', message);
      chatInputEl.value = '';
      try {
        addEvent('发送对话', message);
        const data = await api('/api/chat', { message });
        const intent = data.intent || {};
        const meta = intent.instruction
          ? `${data.llm_used ? data.llm_model : 'local'} · ${intent.instruction}`
          : `${data.llm_used ? data.llm_model : 'local'} · ${intent.action || 'unknown'}`;
        addChat('assistant', data.reply || '', meta);
        if (data.execution) {
          setResult(data.execution);
        }
        if (data.llm_status) {
          setLlmStatus(data.llm_status);
        }
        if (data.fallback_reason) {
          addEvent('LLM 降级', data.fallback_reason, 'failed');
        }
        setStatus(true, data.executed ? '已执行' : '已解析');
        rawEl.textContent = JSON.stringify(data, null, 2);
      } catch (err) {
        setStatus(false, '对话失败');
        addChat('assistant', String(err), 'error');
        addEvent('对话失败', String(err), 'failed');
      } finally {
        chatBtn.disabled = false;
      }
    }

    function connectEvents() {
      if (!window.EventSource) {
        addEvent('实时状态不可用', '当前浏览器不支持 EventSource');
        return;
      }
      const source = new EventSource('/api/events');
      source.onopen = () => setStatus(true, '实时已连接');
      source.onerror = () => setStatus(false, '实时重连中');
      source.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (typeof data.id === 'number' && data.id <= lastEventId) return;
          if (typeof data.id === 'number') lastEventId = data.id;
          const type = data.type || 'message';
          const titleMap = {
            step: '导航步骤',
            feedback: '实时反馈',
            completed: '执行完成',
            failed: '执行失败',
          };
          const cls = type === 'feedback' ? 'feedback' : (type === 'failed' ? 'failed' : (type === 'completed' ? 'completed' : ''));
          addEvent(titleMap[type] || '状态', data.message || JSON.stringify(data), cls);
        } catch (err) {
          addEvent('实时事件', event.data);
        }
      };
    }

    executeBtn.onclick = executeInstruction;
    chatBtn.onclick = chat;
    capBtn.onclick = () => loadCapabilities().catch((err) => {
      setStatus(false, '能力请求失败');
      addEvent('能力请求失败', String(err));
    });
    clearBtn.onclick = () => { timelineEl.innerHTML = ''; rawEl.textContent = '{}'; };
    clearChatBtn.onclick = () => { chatlogEl.innerHTML = ''; };
    instructionEl.addEventListener('keydown', (event) => {
      if ((event.ctrlKey || event.metaKey) && event.key === 'Enter') {
        executeInstruction();
      }
    });
    chatInputEl.addEventListener('keydown', (event) => {
      if ((event.ctrlKey || event.metaKey) && event.key === 'Enter') {
        chat();
      }
    });

    connectEvents();
    loadLlmStatus().catch((err) => {
      setLlmStatus({ status: 'unknown', reason: String(err) });
    });
    loadCapabilities().catch((err) => {
      setStatus(false, '未连接');
      addEvent('初始化失败', String(err));
    });
  </script>
</body>
</html>
"""


class InstructionWebNode(Node):
    def __init__(self) -> None:
        super().__init__('navflex_instruction_web')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('service_timeout', 120.0)
        self.declare_parameter('llm_enabled', True)
        self.declare_parameter('llm_backend', 'openai')
        self.declare_parameter('llm_model', os.environ.get('OPENAI_MODEL', 'gpt-5.5'))
        self.declare_parameter('codex_command', 'codex')
        self.declare_parameter('codex_args', ['exec', '--skip-git-repo-check', '-'])
        self.declare_parameter('openai_api_url', 'https://api.openai.com/v1/responses')
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('openai_api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('openai_proxy', '')
        self.declare_parameter('openai_proxy_env', '')
        self.declare_parameter('llm_timeout', 20.0)
        self.declare_parameter('auto_execute_chat', True)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.service_timeout = float(self.get_parameter('service_timeout').value)
        self.llm_enabled = self._get_bool_parameter('llm_enabled')
        self.llm_backend = str(self.get_parameter('llm_backend').value).strip().lower()
        self.llm_model = str(self.get_parameter('llm_model').value)
        self.codex_command = str(self.get_parameter('codex_command').value)
        self.codex_args = [str(arg) for arg in self.get_parameter('codex_args').value]
        self.openai_api_url = str(self.get_parameter('openai_api_url').value)
        self.llm_timeout = float(self.get_parameter('llm_timeout').value)
        self.auto_execute_chat = self._get_bool_parameter('auto_execute_chat')
        self.openai_api_key_env = str(self.get_parameter('openai_api_key_env').value)
        self.openai_api_key = (
            str(self.get_parameter('openai_api_key').value).strip() or
            os.environ.get(self.openai_api_key_env, '').strip()
        )
        self.openai_proxy_env = str(self.get_parameter('openai_proxy_env').value).strip()
        self.openai_proxy = (
            str(self.get_parameter('openai_proxy').value).strip() or
            self._read_proxy_from_environment(self.openai_proxy_env)
        )
        self.callback_group = ReentrantCallbackGroup()
        self.execute_client = self.create_client(
            ExecuteInstruction, 'navflex_instruction/execute', callback_group=self.callback_group)
        self.cap_client = self.create_client(
            GetCapabilities, 'navflex_instruction/capabilities', callback_group=self.callback_group)
        self.event_sub = self.create_subscription(
            String, 'navflex_instruction/events', self._on_realtime_event, 10,
            callback_group=self.callback_group)
        self.httpd = None
        self.http_thread = None
        self.events_cond = threading.Condition()
        self.events: List[Dict[str, Any]] = []
        self.next_event_id = 0
        self.chat_history: List[Dict[str, str]] = []
        self.last_llm_error = ''
        self._codex_cli_ready_cache = None
        self.get_logger().info(
            "LLM navigation parser: "
            f"enabled={self.llm_enabled} backend='{self.llm_backend}' "
            f"model='{self.llm_model}' codex_command='{self.codex_command}' "
            f"api_key_env='{self.openai_api_key_env}' "
            f"api_key_present={bool(self.openai_api_key)} "
            f"proxy_present={bool(self.openai_proxy)} "
            f"auto_execute_chat={self.auto_execute_chat}")

    def _get_bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ['1', 'true', 'yes', 'on']
        return bool(value)

    def _read_proxy_from_environment(self, explicit_env_name: str) -> str:
        if explicit_env_name:
            return os.environ.get(explicit_env_name, '').strip()
        for name in [
            'HTTPS_PROXY', 'https_proxy',
            'HTTP_PROXY', 'http_proxy',
            'ALL_PROXY', 'all_proxy',
        ]:
            value = os.environ.get(name, '').strip()
            if value:
                return value
        return ''

    def start_http(self) -> None:
        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Navflex instruction web UI listening on http://{self.host}:{self.port}')

    def stop_http(self) -> None:
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.httpd = None
        if self.http_thread is not None:
            self.http_thread.join(timeout=1.0)
            self.http_thread = None

    def execute_instruction(self, instruction: str) -> Dict[str, Any]:
        if not self.execute_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('/navflex_instruction/execute service is not available')
        request = ExecuteInstruction.Request()
        request.instruction = instruction
        future = self.execute_client.call_async(request)
        response = self._wait_future(future, 'execute instruction')
        return self._execute_response_to_dict(response)

    def get_capabilities(self) -> Dict[str, Any]:
        if not self.cap_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('/navflex_instruction/capabilities service is not available')
        future = self.cap_client.call_async(GetCapabilities.Request())
        response = self._wait_future(future, 'get capabilities')
        return {
            'capabilities': list(response.capabilities),
            'examples': list(response.examples),
            'schema': response.schema,
        }

    def _wait_future(self, future, label: str):
        deadline = time.monotonic() + self.service_timeout
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
            if time.monotonic() > deadline:
                raise TimeoutError(f'timed out waiting for {label}')
        if future.result() is None:
            raise RuntimeError(f'{label} returned no response')
        return future.result()

    def _execute_response_to_dict(self, response) -> Dict[str, Any]:
        return {
            'success': bool(response.success),
            'command_type': response.command_type,
            'normalized_command': response.normalized_command,
            'message': response.message,
            'steps': list(response.steps),
            'elapsed_time': {
                'sec': int(response.elapsed_time.sec),
                'nanosec': int(response.elapsed_time.nanosec),
            },
        }

    def chat(self, message: str) -> Dict[str, Any]:
        intent = self._resolve_chat_intent(message)
        execution = None
        executed = False

        if intent.get('action') == 'execute':
            instruction = str(intent.get('instruction', '')).strip()
            if not instruction:
                intent = {
                    'action': 'clarify',
                    'instruction': '',
                    'reply': '我需要更明确的目标、距离或角度。',
                    'confidence': 0.0,
                    'source': intent.get('source', 'local'),
                }
            elif self.auto_execute_chat:
                execution = self.execute_instruction(instruction)
                executed = True
                if not intent.get('reply'):
                    intent['reply'] = (
                        '已发送导航指令。'
                        if execution.get('success') else
                        f"指令已解析，但执行失败：{execution.get('message', '')}")
            else:
                intent['reply'] = f"已解析为指令：{instruction}"

        reply = str(intent.get('reply') or '我还不能确定要执行哪条导航指令。')
        self._remember_chat('user', message)
        self._remember_chat('assistant', reply)

        return {
            'reply': reply,
            'intent': intent,
            'executed': executed,
            'execution': execution,
            'llm_used': intent.get('source') in ['openai', 'codex_cli'],
            'llm_model': self.llm_model if intent.get('source') == 'openai' else intent.get('source', ''),
            'llm_status': self.get_llm_status(),
            'fallback_reason': intent.get('fallback_reason', ''),
        }

    def _resolve_chat_intent(self, message: str) -> Dict[str, Any]:
        fallback_reason = ''
        if not self.llm_enabled:
            fallback_reason = 'llm_enabled is false'
        elif self.llm_backend == 'local':
            fallback_reason = 'llm_backend is local'
        elif self.llm_backend == 'openai' and not self.openai_api_key:
            fallback_reason = f'{self.openai_api_key_env} is not visible to this ROS process'
        else:
            try:
                self.last_llm_error = ''
                if self.llm_backend == 'openai':
                    return self._resolve_chat_intent_with_openai(message)
                if self.llm_backend == 'codex_cli':
                    return self._resolve_chat_intent_with_codex_cli(message)
                fallback_reason = f"unsupported llm_backend '{self.llm_backend}'"
            except Exception as exc:
                sanitized = self._sanitize_secret_text(str(exc))
                fallback_reason = f'{self.llm_backend} request failed: {sanitized}'
                self.last_llm_error = sanitized
                self.get_logger().warn(
                    'LLM intent parsing failed, falling back to local parser: '
                    f'{sanitized}')

        intent = self._resolve_chat_intent_locally(message)
        intent['fallback_reason'] = fallback_reason
        return intent

    def _resolve_chat_intent_locally(self, message: str) -> Dict[str, Any]:
        instruction = self._normalize_chat_command(message)
        command_words = [
            'go', 'goto', 'navigate', 'move', 'forward', 'backward', 'back',
            'rotate', 'turn', 'wait', '去', '到', '前进', '后退', '旋转',
            '左转', '右转', '等待', '停留', '往前', '往后', '向前', '向后',
        ]
        if any(word in instruction.lower() for word in command_words):
            return {
                'action': 'execute',
                'instruction': instruction,
                'reply': f'按本地规则解析为：{instruction}',
                'confidence': 0.55,
                'source': 'local',
            }
        return {
            'action': 'clarify',
            'instruction': '',
            'reply': '请告诉我目标点、命名地点、移动距离或旋转角度。',
            'confidence': 0.0,
            'source': 'local',
        }

    def _normalize_chat_command(self, message: str) -> str:
        text = message.strip()
        replacements = {
            '请你': '', '请': '', '帮我': '', '机器人': '', '小车': '',
            '往前走': '前进', '向前走': '前进', '往后走': '后退', '向后走': '后退',
            '半米': '0.5米', '一米': '1米', '两米': '2米', '三米': '3米',
            '一秒': '1秒', '两秒': '2秒', '三秒': '3秒',
            '半圈': '180度',
        }
        for old, new in replacements.items():
            text = text.replace(old, new)
        return text.strip()

    def _resolve_chat_intent_with_openai(self, message: str) -> Dict[str, Any]:
        schema = {
            'type': 'object',
            'additionalProperties': False,
            'properties': {
                'action': {
                    'type': 'string',
                    'enum': ['execute', 'clarify', 'answer'],
                },
                'instruction': {'type': 'string'},
                'reply': {'type': 'string'},
                'confidence': {
                    'type': 'number',
                    'minimum': 0.0,
                    'maximum': 1.0,
                },
            },
            'required': ['action', 'instruction', 'reply', 'confidence'],
        }
        payload = {
            'model': self.llm_model,
            'instructions': self._llm_instructions(),
            'input': self._llm_input(message),
            'text': {
                'format': {
                    'type': 'json_schema',
                    'name': 'navflex_navigation_intent',
                    'strict': True,
                    'schema': schema,
                },
            },
        }
        try:
            data = self._post_openai_response(payload)
        except RuntimeError as exc:
            payload.pop('text', None)
            payload['instructions'] += '\n只输出一个 JSON 对象，不要输出 Markdown。'
            try:
                data = self._post_openai_response(payload)
            except Exception:
                raise exc

        text = self._extract_response_text(data)
        intent = self._parse_json_object(text)
        action = intent.get('action', 'clarify')
        if action not in ['execute', 'clarify', 'answer']:
            action = 'clarify'
        return {
            'action': action,
            'instruction': str(intent.get('instruction', '')).strip(),
            'reply': str(intent.get('reply', '')).strip(),
            'confidence': float(intent.get('confidence', 0.0) or 0.0),
            'source': 'openai',
        }

    def _resolve_chat_intent_with_codex_cli(self, message: str) -> Dict[str, Any]:
        if shutil.which(self.codex_command) is None:
            raise RuntimeError(f"codex command not found: {self.codex_command}")

        prompt = (
            f'{self._llm_instructions()}\n\n'
            f'{self._llm_input(message)}\n\n'
            '只输出 JSON 对象，不要输出 Markdown，不要解释。'
        )
        completed = subprocess.run(
            [self.codex_command] + self.codex_args,
            input=prompt,
            text=True,
            capture_output=True,
            timeout=self.llm_timeout,
            cwd=os.getcwd(),
            check=False,
        )
        output = (completed.stdout or '').strip()
        error = (completed.stderr or '').strip()
        if completed.returncode != 0:
            raise RuntimeError(
                f"codex_cli exited with code {completed.returncode}: "
                f"{self._sanitize_secret_text(error or output)}")
        intent = self._parse_json_object(output)
        action = intent.get('action', 'clarify')
        if action not in ['execute', 'clarify', 'answer']:
            action = 'clarify'
        return {
            'action': action,
            'instruction': str(intent.get('instruction', '')).strip(),
            'reply': str(intent.get('reply', '')).strip(),
            'confidence': float(intent.get('confidence', 0.0) or 0.0),
            'source': 'codex_cli',
        }

    def _check_codex_cli_ready(self) -> str:
        if self._codex_cli_ready_cache is not None:
            return self._codex_cli_ready_cache
        if shutil.which(self.codex_command) is None:
            self._codex_cli_ready_cache = f"codex command not found: {self.codex_command}"
            return self._codex_cli_ready_cache
        try:
            completed = subprocess.run(
                [self.codex_command, '--help'],
                text=True,
                capture_output=True,
                timeout=5.0,
                check=False,
            )
        except Exception as exc:
            self._codex_cli_ready_cache = str(exc)
            return self._codex_cli_ready_cache
        if completed.returncode != 0:
            self._codex_cli_ready_cache = self._sanitize_secret_text(
                (completed.stderr or completed.stdout or '').strip())
            return self._codex_cli_ready_cache
        self._codex_cli_ready_cache = ''
        return ''

    def _post_openai_response(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(
            self.openai_api_url,
            data=body,
            headers={
                'Authorization': f'Bearer {self.openai_api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        opener = urllib.request.build_opener()
        if self.openai_proxy:
            opener = urllib.request.build_opener(
                urllib.request.ProxyHandler({
                    'http': self.openai_proxy,
                    'https': self.openai_proxy,
                }))
        try:
            with opener.open(request, timeout=self.llm_timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            detail = self._sanitize_secret_text(detail)
            raise RuntimeError(f'OpenAI API HTTP {exc.code}: {detail}') from exc

    def _extract_response_text(self, data: Dict[str, Any]) -> str:
        if data.get('output_text'):
            return str(data['output_text'])
        chunks = []
        for item in data.get('output', []):
            for content in item.get('content', []):
                if isinstance(content, dict):
                    if content.get('text'):
                        chunks.append(str(content['text']))
                    elif content.get('type') == 'output_text' and content.get('content'):
                        chunks.append(str(content['content']))
        if chunks:
            return ''.join(chunks)
        raise RuntimeError('OpenAI response did not contain text output')

    def _parse_json_object(self, text: str) -> Dict[str, Any]:
        text = text.strip()
        if text.startswith('```'):
            text = text.strip('`')
            if text.lower().startswith('json'):
                text = text[4:].strip()
        start = text.find('{')
        end = text.rfind('}')
        if start >= 0 and end > start:
            text = text[start:end + 1]
        return json.loads(text)

    def _sanitize_secret_text(self, text: str) -> str:
        return re.sub(r'sk-[A-Za-z0-9_\-]{8,}', 'sk-***', text)

    def _llm_instructions(self) -> str:
        return (
            '你是 Navflex 机器人导航指令解析器。你的任务是把中文或英文对话'
            '转换成一条安全、明确、单步的 Navflex 文本指令。\n'
            '允许的指令形式：\n'
            '- 坐标导航：go to <x> <y> [yawdeg]\n'
            '- 命名地点：goto <name>\n'
            '- 相对移动：forward <meters>m 或 backward <meters>m\n'
            '- 旋转：rotate <degrees>deg，左转为正，右转为负\n'
            '- 等待：wait <seconds>\n'
            '如果用户没有给出目标、距离、角度或命名地点，action 必须是 clarify。'
            '不要自己编造坐标；地点名未知时也可以输出 goto <地点名>，由后端配置决定是否可用。'
            '不要输出多步指令；如果用户要求多步，先请求澄清。'
        )

    def _llm_input(self, message: str) -> str:
        history = '\n'.join(
            f"{item['role']}: {item['content']}" for item in self.chat_history[-8:])
        return (
            f'最近对话：\n{history or "(无)"}\n\n'
            f'用户最新输入：{message}\n\n'
            '返回 JSON：{"action":"execute|clarify|answer",'
            '"instruction":"...", "reply":"...", "confidence":0.0}'
        )

    def _remember_chat(self, role: str, content: str) -> None:
        self.chat_history.append({'role': role, 'content': content})
        if len(self.chat_history) > 16:
            self.chat_history = self.chat_history[-16:]

    def get_llm_status(self) -> Dict[str, Any]:
        if not self.llm_enabled:
            status = 'disabled'
            reason = 'llm_enabled is false'
        elif self.llm_backend == 'local':
            status = 'local'
            reason = 'llm_backend is local'
        elif self.llm_backend == 'openai' and not self.openai_api_key:
            status = 'missing_key'
            reason = f'{self.openai_api_key_env} is not visible to this ROS process'
        elif self.llm_backend == 'codex_cli' and self._check_codex_cli_ready():
            status = 'codex_cli_error'
            reason = self._check_codex_cli_ready()
        elif self.last_llm_error:
            status = 'fallback'
            reason = self.last_llm_error
        else:
            status = 'ready'
            reason = ''
        return {
            'status': status,
            'enabled': self.llm_enabled,
            'backend': self.llm_backend,
            'api_key_present': bool(self.openai_api_key),
            'api_key_env': self.openai_api_key_env,
            'proxy_present': bool(self.openai_proxy),
            'proxy_env': self.openai_proxy_env,
            'model': self.llm_model,
            'codex_command': self.codex_command,
            'codex_command_present': shutil.which(self.codex_command) is not None,
            'codex_args': self.codex_args,
            'auto_execute_chat': self.auto_execute_chat,
            'last_error': self.last_llm_error,
            'reason': reason,
        }

    def _on_realtime_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            event = {'type': 'message', 'message': msg.data, 'stamp': time.time()}
        with self.events_cond:
            event['id'] = self.next_event_id
            self.next_event_id += 1
            self.events.append(event)
            self.events = self.events[-300:]
            self.events_cond.notify_all()

    def wait_events(self, last_event_id: int, timeout: float = 15.0) -> List[Dict[str, Any]]:
        deadline = time.monotonic() + timeout
        with self.events_cond:
            while rclpy.ok():
                ready = [event for event in self.events if event.get('id', -1) > last_event_id]
                if ready:
                    return ready
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return []
                self.events_cond.wait(timeout=remaining)
        return []

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/':
                    self._send_html(INDEX_HTML)
                    return
                if parsed.path == '/api/capabilities':
                    self._handle_json(lambda: node.get_capabilities())
                    return
                if parsed.path == '/api/llm_status':
                    self._handle_json(lambda: node.get_llm_status())
                    return
                if parsed.path == '/api/events':
                    self._handle_events()
                    return
                self._send_json({'error': 'not found'}, status=404)

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path == '/api/execute':
                    self._handle_json(self._execute)
                    return
                if parsed.path == '/api/chat':
                    self._handle_json(self._chat)
                    return
                self._send_json({'error': 'not found'}, status=404)

            def log_message(self, fmt, *args):
                node.get_logger().debug(fmt % args)

            def _execute(self):
                body = self._read_json()
                instruction = str(body.get('instruction', '')).strip()
                if not instruction:
                    raise ValueError('instruction is empty')
                return node.execute_instruction(instruction)

            def _chat(self):
                body = self._read_json()
                message = str(body.get('message', '')).strip()
                if not message:
                    raise ValueError('message is empty')
                return node.chat(message)

            def _handle_json(self, fn):
                try:
                    self._send_json(fn())
                except Exception as exc:
                    node.get_logger().warn(f'Web request failed: {exc}')
                    self._send_json({'error': str(exc)}, status=500)

            def _handle_events(self):
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                self.wfile.write(b': connected\n\n')
                self.wfile.flush()
                last_event_id = -1
                while rclpy.ok():
                    try:
                        for event in node.wait_events(last_event_id):
                            last_event_id = int(event.get('id', last_event_id))
                            data = json.dumps(event, ensure_ascii=False)
                            self.wfile.write(f'id: {last_event_id}\n'.encode('utf-8'))
                            self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                            self.wfile.flush()
                        self.wfile.write(b': keepalive\n\n')
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return

            def _read_json(self):
                length = int(self.headers.get('Content-Length', '0'))
                raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
                return json.loads(raw or '{}')

            def _send_html(self, html):
                data = html.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _send_json(self, payload, status=200):
                data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InstructionWebNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    node.start_http()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_http()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
