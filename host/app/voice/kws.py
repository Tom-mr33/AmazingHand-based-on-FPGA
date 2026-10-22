"""离线中文命令词识别 —— Vosk 小模型 + 语法限词表。

设计：
- **语法限词**：Vosk ``SetGrammar`` 把识别空间限制到命令词表内
  （抓/放/停/复位 + 15 类物品中文名），显著提升短命令词召回——
  这是"命令词识别"而非"自由转写"的正确用法；
- 输出结构化指令 :class:`Command`（action + target），对齐
  ``main.VoiceSource`` 协议——``main.py`` 的 MockVoice 换成本类即接入真实语音；
- ``transcribe_wav``：文件驱动入口（PC 无麦克风时的测试/演示路径）。

模型：vosk-model-small-cn-0.22（42MB，A53 实测可用），下载脚本
``tools/download_vosk_model.py``，存放 ``host/models/``（不入 Git）。

命令词表验收（F1）：词表内识别可靠、端到端 ≤2s——端到端延迟含
音频分片（0.1s 级）+ 推理（A53 上小模型 ~0.3×RT）+ 解析（µs 级）。
"""

from __future__ import annotations

import json
import wave
from dataclasses import dataclass
from pathlib import Path

#: 15 类物品中文名 → 内部物品名（grasp_table.json 键）
ZH_OBJECTS: dict[str, str] = {
    "香蕉": "banana", "苹果": "apple", "橙子": "orange", "橘子": "orange",
    "瓶子": "bottle", "杯子": "cup", "水杯": "cup", "玻璃杯": "wine_glass",
    "碗": "bowl", "鼠标": "mouse", "遥控器": "remote", "键盘": "keyboard",
    "手机": "cell_phone", "书": "book", "剪刀": "scissors",
    "玩具熊": "teddy_bear", "牙刷": "toothbrush",
}
#: 动作词 → action
ZH_ACTIONS: dict[str, str] = {
    "抓": "grasp", "拿": "grasp", "捡": "grasp",
    "放": "release", "松": "release", "开": "release",
    "停": "stop", "复位": "reset", "重置": "reset",
}

DEFAULT_MODEL_DIR = Path(__file__).resolve().parents[2] / "models/vosk-model-small-cn-0.22"


@dataclass(frozen=True)
class Command:
    """结构化指令（与 main.Command 契约一致；独立定义避免循环依赖）。"""

    action: str  # grasp / release / stop / reset
    target: str = ""


def grammar_words() -> list[str]:
    """Vosk 语法词表（识别空间=命令词全组合）。"""
    words = list(ZH_ACTIONS)
    words += list(ZH_OBJECTS)
    # 常见填充音节，防止误吞字
    words += ["个", "把", "起", "来", "下", "那个", "这个"]
    return words


def parse_zh_command(text: str) -> Command | None:
    """识别文本 → 结构化指令。动作取最先命中的动作词，目标取物品词。"""
    action = None
    for w, a in ZH_ACTIONS.items():
        if w in text:
            action = a
            break
    if action is None:
        return None
    target = ""
    if action == "grasp":
        for w, item in ZH_OBJECTS.items():
            if w in text:
                target = item
                break
    return Command(action, target)


class ZhCommandKws:
    """Vosk 命令词识别器（VoiceSource 协议实现）。"""

    def __init__(self, model_dir: Path | str = DEFAULT_MODEL_DIR,
                 sample_rate: int = 16000, use_grammar: bool = True):
        from vosk import Model, KaldiRecognizer  # 延迟导入：mock 模式无需 vosk

        model_dir = Path(model_dir)
        if not model_dir.exists():
            raise FileNotFoundError(
                f"vosk 模型缺失: {model_dir}（运行 tools/download_vosk_model.py）"
            )
        self._model = Model(str(model_dir))
        self._rate = sample_rate
        self._grammar = json.dumps(grammar_words(), ensure_ascii=False)
        self._use_grammar = use_grammar
        self._rec: object | None = None
        self.reset()

    def reset(self) -> None:
        """新建识别器（每条命令独立会话，避免跨命令状态串扰）。"""
        from vosk import KaldiRecognizer

        self._rec = KaldiRecognizer(self._model, self._rate)
        if self._use_grammar:
            self._rec.SetGrammar(self._grammar)

    # ------------------------------------------------------------------
    # 流式接口（麦克风路径，树莓派使用）
    # ------------------------------------------------------------------

    def feed(self, pcm_bytes: bytes) -> Command | None:
        """喂一段 16kHz mono int16 PCM；final 文本出现时返回解析结果。"""
        if self._rec is None:
            self.reset()
        if self._rec.AcceptWaveform(pcm_bytes):
            return self._parse_final(json.loads(self._rec.Result()))
        return None

    def finish(self) -> Command | None:
        """音频结束：取 final 结果。"""
        if self._rec is None:
            return None
        cmd = self._parse_final(json.loads(self._rec.FinalResult()))
        self.reset()
        return cmd

    # ------------------------------------------------------------------
    # 文件驱动接口（PC 测试/演示）
    # ------------------------------------------------------------------

    def transcribe_wav(self, wav_path: Path | str) -> str:
        """整段 wav（16k mono int16）→ 识别文本（测试/演示用）。"""
        with wave.open(str(wav_path), "rb") as wf:
            if wf.getframerate() != self._rate or wf.getnchannels() != 1:
                raise ValueError(f"wav must be {self._rate}Hz mono int16")
            self.reset()
            while True:
                chunk = wf.readframes(4000)
                if not chunk:
                    break
                self._rec.AcceptWaveform(chunk)
            return json.loads(self._rec.FinalResult()).get("text", "")

    def command_from_wav(self, wav_path: Path | str) -> Command | None:
        """wav → 结构化指令（语音→命令端到端）。"""
        return parse_zh_command(self.transcribe_wav(wav_path))

    # ------------------------------------------------------------------

    def _parse_final(self, result: dict) -> Command | None:
        text = (result.get("text") or "").replace(" ", "")
        return parse_zh_command(text) if text else None


class MicVoiceSource:
    """麦克风 → Kws → 命令队列（main.VoiceSource 协议实现，板端 run_app.sh 使用）。

    依赖 pyaudio（树莓派/PC 安装）；无麦克风/依赖时构造报错，
    PC 演示请用 main.MockVoice。
    """

    def __init__(self, kws: ZhCommandKws, rate: int = 16000, chunk_ms: int = 100):
        import queue
        import threading

        self._kws = kws
        self._rate = rate
        self._chunk = rate * chunk_ms // 1000
        self._q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self) -> None:  # pragma: no cover —— 板端运行路径
        import pyaudio

        pa = pyaudio.PyAudio()
        stream = pa.open(self._rate, 1, pyaudio.paInt16, input=True,
                         frames_per_buffer=self._chunk)
        while not self._stop.is_set():
            pcm = stream.read(self._chunk, exception_on_overflow=False)
            cmd = self._kws.feed(pcm)
            if cmd is not None:
                self._q.put(cmd)
        stream.close()
        pa.terminate()

    def poll_command(self) -> Command | None:
        try:
            return self._q.get_nowait()
        except Exception:  # noqa: BLE001 —— queue.Empty
            return None

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
