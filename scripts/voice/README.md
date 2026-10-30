# Voice commands (openWakeWord + Whisper)

Say **hey jarvis**, then one command. The robot goes back to waiting for the wake phrase. Uses [openWakeWord](https://github.com/dscripka/openWakeWord) (pretrained **hey jarvis** model — no recording your voice) and [Whisper](https://pypi.org/project/openai-whisper/) ``tiny.en`` on the Pi CPU. Does not use the camera or ``scripts/xgo_robot``.

## Setup on the robot

```bash
cd ~/project/xgo_robot_kit_playing_fetch
python3 -m pip install --user --no-cache-dir -r scripts/voice/requirements.txt
```

If the disk is full (`df -h /`), purge the pip cache and delete old AVI files, then retry. First listen run downloads the small hey-jarvis weights.

## Listen

Stop teleop, movement tests, and any other process on ``/dev/ttyAMA0``.

```bash
cd ~/project/xgo_robot_kit_playing_fetch
python3 scripts/voice/voice.py listen
```

After the safety prompt the dog connects once (brief ``reset()`` twitch). Then:

You can say it as **one phrase**: ``hey jarvis sit``. The listener keeps the last two seconds of audio so ``sit`` is not lost after the wake fires. You can also pause after ``hey jarvis``.

If it still misses, lower the threshold: ``python3 scripts/voice/voice.py listen --threshold 0.25``

| After wake, say | Robot |
| --- | --- |
| stop | Halt gait |
| sit / lie down / stand / wave / stretch / handshake | Preset ``action()`` |
| go forward / go back | ``move_x(±10)`` for 1 s, then ``stop()`` |
| slide left / slide right | ``move_y(±8)`` for 1 s, then ``stop()`` |
| turn left / turn right | ``turn(±30)`` for 1 s, then ``stop()`` |

``chase …`` and ``walk N meters`` print “not wired” and do not move. Unknown phrases do **not** call ``reset()``.

Default threshold is ``0.35``. Raise it if random speech triggers the dog; lower it if ``hey jarvis`` is ignored.

## Tests (laptop or Pi, no mic)

```bash
python3 -m unittest discover -s scripts/voice -p 'test_*.py'
```
