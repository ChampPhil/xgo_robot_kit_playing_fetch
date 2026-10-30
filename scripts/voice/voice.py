"""Wake-word loop, local Whisper, and one-shot XGO Lite commands.

Purpose:
    Run on the robot's Raspberry Pi. ``listen`` waits for the pretrained
    openWakeWord model ``hey jarvis`` (anyone can say it; no reference clips),
    records the next few seconds, transcribes with Whisper ``tiny.en`` on CPU,
    maps the text to a single motion or preset action, then goes back to
    waiting. There is no command queue and no camera chase.

Key terms:
    - Wake phrase: ``hey jarvis``. Detected by openWakeWord's pretrained
      neural model, not by comparing against your own WAV recordings.
    - Command clip: Audio captured immediately after a wake detection; only
      this clip is sent to Whisper.
    - Stride command: ``move_x`` / ``move_y`` / ``turn`` with a speed value;
      the robot keeps that gait until ``stop()`` — same semantics as
      ``scripts/movement_tests/basic_movement.py``.
    - Preset action: ``action(id)`` plays a firmware gesture; the call returns
      before the motion finishes, so we sleep for the documented duration.

Non-obvious behavior:
    - Phrase matching uses longest substring first so "turn left" is not read
      as "left".
    - Unknown transcripts, ``chase …``, and ``walk N meters`` print a message
      and do not call ``reset()``.
    - ``XGO()`` on connect runs ``reset()`` once; expect a brief startup twitch.
    - Mic frames are 1280 samples (80 ms at 16 kHz), which openWakeWord expects.
    - A rolling 2 s buffer is prepended to the command clip so "hey jarvis sit"
      in one breath still contains "sit" for Whisper.

Exports:
    - parse_command, explain_no_command, execute_command, load_wake_model,
      run_voice, build_parser, main.
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
import time
import queue
from importlib import import_module
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_THRESHOLD = 0.35
PRE_ROLL_S = 2.0
POST_ROLL_S = 2.0
MOVE_DURATION_S = 1.0
X_STEP = 10
Y_STEP = 8
TURN_SPEED = 30
WHISPER_MODEL = "tiny.en"
SAMPLE_RATE = 16000
WAKE_FRAME = 1280
WAKE_COOLDOWN_S = 1.0

# Longest phrases first so partial overlaps resolve correctly.
PHRASE_TO_COMMAND: tuple[tuple[str, str], ...] = (
    ("go forward", "go_forward"),
    ("go back", "go_back"),
    ("slide left", "slide_left"),
    ("slide right", "slide_right"),
    ("turn left", "turn_left"),
    ("turn right", "turn_right"),
    ("lie down", "lie_down"),
    ("handshake", "handshake"),
    ("stretch", "stretch"),
    ("stand up", "stand"),
    ("stand", "stand"),
    ("forward", "go_forward"),
    ("backward", "go_back"),
    ("stop", "stop"),
    ("wave", "wave"),
    ("sit", "sit"),
)

GESTURE_ACTION: dict[str, tuple[int, float]] = {
    "sit": (12, 5.0),
    "lie_down": (1, 3.0),
    "stand": (2, 3.0),
    "wave": (13, 7.0),
    "stretch": (14, 10.0),
    "handshake": (19, 10.0),
}

_CHASE_RE = re.compile(r"\bchase\b", re.I)
_WALK_METERS_RE = re.compile(r"\bwalk\s+\d+(?:\.\d+)?\s*meters?\b", re.I)


def _log(message: str) -> None:
    """Print immediately so SSH does not look hung during Whisper.

    :param message: Line to show on the terminal.
    :returns: None.
    """

    print(message, flush=True)


def _normalize(text: str) -> str:
    """Lowercase and collapse whitespace for substring phrase matching.

    :param text: Raw Whisper transcript.
    :returns: Normalized string.
    """

    cleaned = re.sub(r"[^\w\s']", " ", text.lower())
    return " ".join(cleaned.split())


def parse_command(text: str) -> str | None:
    """Map a transcript to an internal command name, or None if no phrase matched.

    :param text: Whisper output (English).
    :returns: Command key such as ``go_forward``, or None.
    """

    normalized = _normalize(text)
    if not normalized:
        return None
    best: tuple[int, str] | None = None
    for phrase, command in PHRASE_TO_COMMAND:
        index = normalized.find(phrase)
        if index < 0:
            continue
        if best is None or len(phrase) > best[0]:
            best = (len(phrase), command)
    return best[1] if best else None


def explain_no_command(text: str) -> None:
    """Print why a transcript did not run a motion (no ``reset()``).

    :param text: Raw transcript.
    :returns: None.
    """

    if _CHASE_RE.search(text):
        _log("Chase is not wired yet. Use movement_tests or teleop for now.")
        return
    if _WALK_METERS_RE.search(text):
        _log("Walk N meters is not wired yet. Stride commands are timed, not odometry.")
        return
    _log(f"No command matched: {text!r}")


def execute_command(dog: object, command: str) -> None:
    """Run exactly one command and ensure gait commands end in ``stop()``.

    :param dog: Connected ``XGO`` instance from xgolib.
    :param command: Internal name from ``parse_command``.
    :returns: None.
    """

    if command == "stop":
        dog.stop()
        _log("Stopped.")
        return

    gesture = GESTURE_ACTION.get(command)
    if gesture is not None:
        action_id, wait_s = gesture
        dog.action(action_id)
        time.sleep(wait_s)
        _log(f"Finished preset: {command}.")
        return

    timed: tuple[str, float] | None = None
    if command == "go_forward":
        timed = ("x", float(X_STEP))
    elif command == "go_back":
        timed = ("x", -float(X_STEP))
    elif command == "slide_left":
        timed = ("y", float(Y_STEP))
    elif command == "slide_right":
        timed = ("y", -float(Y_STEP))
    elif command == "turn_left":
        timed = ("turn", float(TURN_SPEED))
    elif command == "turn_right":
        timed = ("turn", -float(TURN_SPEED))

    if timed is not None:
        axis, value = timed
        try:
            if axis == "x":
                dog.move_x(value)
            elif axis == "y":
                dog.move_y(value)
            else:
                dog.turn(value)
            time.sleep(MOVE_DURATION_S)
        finally:
            dog.stop()
        _log(f"Finished timed move: {command}.")
        return

    _log(f"Internal error: unknown command {command!r}")


def _wake_score(scores: dict) -> float:
    """Return the highest jarvis score from an openWakeWord prediction dict.

    :param scores: Model name to confidence in ``[0, 1]``.
    :returns: Best matching score, or 0 if none.
    """

    jarvis = [float(value) for name, value in scores.items() if "jarvis" in name.lower()]
    if jarvis:
        return max(jarvis)
    return max((float(v) for v in scores.values()), default=0.0)


def load_wake_model() -> object:
    """Download pretrained models if needed and load only hey jarvis.

    :returns: openWakeWord ``Model`` instance.
    :raises RuntimeError: If the hey jarvis weights cannot be found.
    """

    import openwakeword
    from openwakeword.model import Model

    # First run fetches the small .tflite/.onnx files into the package resources.
    openwakeword.utils.download_models()
    resources = Path(openwakeword.__file__).resolve().parent / "resources" / "models"
    onnx = sorted(resources.glob("hey_jarvis*.onnx"))
    tflite = sorted(resources.glob("hey_jarvis*.tflite"))
    if onnx:
        _log(f"Wake model: {onnx[-1].name} (onnx)")
        return Model(wakeword_models=[str(onnx[-1])], inference_framework="onnx")
    if tflite:
        _log(f"Wake model: {tflite[-1].name} (tflite)")
        return Model(wakeword_models=[str(tflite[-1])], inference_framework="tflite")
    raise RuntimeError(
        "openWakeWord hey_jarvis weights missing after download_models(). "
        "Check network and pip install openwakeword."
    )


def pick_input_device() -> int | None:
    """Prefer the WM8960 mic over HDMI/default PortAudio guesses.

    :returns: sounddevice index, or None to use the library default.
    """

    import sounddevice as sd

    devices = sd.query_devices()
    for index, info in enumerate(devices):
        channels = int(info.get("max_input_channels") or 0)
        name = str(info.get("name") or "").lower()
        if channels > 0 and "wm8960" in name:
            return index
    return None


def _frames_to_int16(indata: object) -> object:
    """Convert a sounddevice callback block to int16 mono PCM.

    :param indata: Float or int callback array.
    :returns: 1-D int16 samples.
    """

    import numpy as np

    raw = np.asarray(indata)
    if raw.ndim > 1:
        raw = raw.mean(axis=1)
    if raw.dtype == np.int16:
        return raw.copy()
    clipped = np.clip(raw.astype(np.float32), -1.0, 1.0)
    return (clipped * 32767.0).astype(np.int16)


def collect_post_roll(audio_q: queue.Queue, seconds: float) -> list:
    """Pull extra frames from the callback queue until ``seconds`` elapse.

    :param audio_q: Queue filled by the sounddevice callback.
    :param seconds: How long to keep recording after wake.
    :returns: List of int16 frames (may be short if the mic stalls).
    """

    chunks: list = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            chunks.append(audio_q.get(timeout=max(0.1, remaining)))
        except queue.Empty:
            _log("Mic did not deliver more frames; using what we have.")
            break
    return chunks


def transcribe_path(model: object, wav_path: Path) -> str:
    """Run Whisper on one WAV file.

    :param model: Loaded ``whisper`` model.
    :param wav_path: 16 kHz mono file.
    :returns: Stripped transcript text.
    """

    result = model.transcribe(str(wav_path), fp16=False, language="en")
    return (result.get("text") or "").strip()


def handle_command_clip(dog: object, whisper_model: object, audio: object) -> None:
    """Write a clip, transcribe it, and run at most one command.

    :param dog: Connected ``XGO`` instance.
    :param whisper_model: Loaded Whisper model.
    :param audio: int16 mono samples at 16 kHz.
    :returns: None.
    """

    import soundfile as sf

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as handle:
        temp_path = Path(handle.name)
    try:
        sf.write(str(temp_path), audio, SAMPLE_RATE, subtype="PCM_16")
        _log("Running Whisper (often 10–40s on this Pi — wait for Heard)...")
        text = transcribe_path(whisper_model, temp_path)
    finally:
        temp_path.unlink(missing_ok=True)
    _log(f"Heard: {text!r}")
    command = parse_command(text)
    if command:
        execute_command(dog, command)
    else:
        explain_no_command(text)


def run_voice(
    dog: object,
    threshold: float,
    *,
    whisper_model_name: str = WHISPER_MODEL,
) -> None:
    """Block on openWakeWord hey jarvis; on each wake, transcribe one clip.

    :param dog: Connected ``XGO`` instance.
    :param threshold: Score in ``[0, 1]``; default 0.35 (0.5 missed a lot on this mic).
    :param whisper_model_name: Whisper checkpoint name.
    :returns: None; runs until Ctrl+C.
    """

    import numpy as np
    import sounddevice as sd
    import whisper

    wake = load_wake_model()
    _log(f"Loading Whisper {whisper_model_name} (CPU)...")
    whisper_model = whisper.load_model(whisper_model_name)
    device = pick_input_device()
    if device is not None:
        _log(f"Mic: {sd.query_devices(device)['name']}")
    _log(
        f"Say 'hey jarvis', wait for wake, then say the command immediately "
        f"(threshold={threshold:g}). Whisper is slow. Ctrl+C to quit."
    )

    pre_frames = int(PRE_ROLL_S * SAMPLE_RATE / WAKE_FRAME)
    history: list = []
    audio_q: queue.Queue = queue.Queue(maxsize=250)

    def on_audio(indata, frames, time_info, status) -> None:
        """Push each mic block onto the queue; drop oldest if the queue is full."""
        del frames, time_info
        if status:
            _log(f"Mic status: {status}")
        frame = _frames_to_int16(indata)
        try:
            audio_q.put_nowait(frame)
        except queue.Full:
            try:
                audio_q.get_nowait()
            except queue.Empty:
                pass
            audio_q.put_nowait(frame)

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=WAKE_FRAME,
        device=device,
        callback=on_audio,
    ) as stream:
        while True:
            try:
                frame = audio_q.get(timeout=1.0)
            except queue.Empty:
                continue
            history.append(frame)
            if len(history) > pre_frames:
                history.pop(0)
            scores = wake.predict(frame)
            score = _wake_score(scores)
            if score < threshold:
                continue
            _log(f"Wake detected (score {score:.2f}). SAY THE COMMAND NOW (recording {POST_ROLL_S:g}s)...")
            post = collect_post_roll(audio_q, POST_ROLL_S)
            audio = np.concatenate(history + post) if post else np.concatenate(history)
            _log(f"Captured {audio.size / SAMPLE_RATE:.1f}s of audio.")
            # Whisper holds the GIL for a long time; leave the mic stopped so
            # PortAudio does not overflow and the queue does not store ~20 s.
            stream.stop()
            handle_command_clip(dog, whisper_model, audio)
            while True:
                try:
                    audio_q.get_nowait()
                except queue.Empty:
                    break
            history.clear()
            if hasattr(wake, "reset"):
                wake.reset()
            stream.start()
            time.sleep(WAKE_COOLDOWN_S)


def build_parser() -> argparse.ArgumentParser:
    """Build the ``listen`` subcommand.

    :returns: Configured argument parser.
    """

    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    listen = sub.add_parser("listen", help="hey jarvis, then one voice command")
    listen.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    listen.add_argument(
        "--threshold",
        type=float,
        default=DEFAULT_THRESHOLD,
        help=f"openWakeWord score 0–1 (default: {DEFAULT_THRESHOLD})",
    )
    listen.add_argument("--yes", action="store_true", help="Skip the safety confirmation prompt")
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entry: run the listen loop.

    :param argv: Optional argument list for tests.
    :returns: Process exit code.
    """

    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        xgolib = import_module("xgolib")
    except ImportError:
        print("xgolib is missing; pip3 install --user xgolib (or recreate .venv).", file=sys.stderr)
        return 2

    if not args.yes:
        try:
            input(
                "Clear a level area around the robot. "
                "Press Enter to start voice listen (Ctrl+C to cancel)... "
            )
        except EOFError:
            parser.error("confirmation requires a terminal; use --yes only when it is safe")

    dog = xgolib.XGO(port=args.port, version="xgolite")
    try:
        run_voice(dog, args.threshold)
    except KeyboardInterrupt:
        print("\nVoice listen stopped.")
    finally:
        dog.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
