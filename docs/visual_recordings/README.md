# Visual recordings

Phone videos of the robot (full-quality demo clips). Name: `<What>_<Detail>[_<date>].mp4`.

| File | Date | Shows |
| --- | --- | --- |
| `Chasing_A_Person_Successfully.mp4` | 2026-10-30 | **Person chase**: robot tracks and follows a person with YOLO. |
| `Fetch_Pickup_Success_2026-10-30.mp4` | 2026-10-30 | **Successful autonomous pickup**, seen from above: F → reset, align to the purple block (18 moves), replay of the taught pickup; block gripped and lifted. Trial 1 of the 2026-10-30 series, ~60 s; see `docs/PROGRESS.md`. |
| `Voice_Command_Forward_Move.mp4` | 2026-10 | Voice command: move forward |
| `Voice_Command_Handshake.mp4` | 2026-10 | Voice command: handshake |
| `Voice_Command_Lie_Down.mp4` | 2026-10 | Voice command: lie down |
| `Voice_Command_Sit.mp4` | 2026-10 | Voice command: sit |
| `Voice_Command_Stand_Up.mp4` | 2026-10 | Voice command: stand up |
| `Voice_Command_Stretch.mp4` | 2026-10 | Voice command: stretch |
| `Voice_Command_Turn_Left.mp4` | 2026-10 | Voice command: turn left |
| `Voice_Command_Wave.mp4` | 2026-10 | Voice command: wave |

To add one: `ffmpeg -i in.MOV -vf "scale=720:1280:force_original_aspect_ratio=decrease,fps=30"
-c:v libx264 -preset slow -crf 26 -pix_fmt yuv420p -c:a aac -b:a 96k -movflags +faststart out.mp4`
