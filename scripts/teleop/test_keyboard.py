"""Offline checks for keyboard control; no window or robot required."""

import unittest

import gamepad_sender as sender
import keyboard_input as kb


def message(*keys):
    state = kb.keys_to_state(set(keys))
    return sender.make_message(state, sender.Arming(deadman=True), 0.0, 0)


class KeyMappingTests(unittest.TestCase):
    def test_no_keys_means_no_motion(self):
        command = message()
        self.assertEqual((command["x"], command["y"], command["yaw"]), (0.0, 0.0, 0.0))
        self.assertFalse(command["enable"])
        self.assertFalse(command["arm"])

    def test_walk_keys_at_half_speed_and_shift_for_full(self):
        self.assertEqual((message("w")["x"], message("w")["enable"]), (0.5, True))
        self.assertEqual(message("s")["x"], -0.5)
        self.assertEqual(message("w", "left shift")["x"], 1.0)
        self.assertEqual(message("a")["y"], 0.5)  # side-step left
        self.assertEqual(message("d", "right shift")["y"], -1.0)
        self.assertEqual(message("q")["yaw"], 0.5)  # turn left
        self.assertEqual(message("e")["yaw"], -0.5)

    def test_opposite_keys_cancel(self):
        self.assertEqual(message("w", "s")["x"], 0.0)
        self.assertEqual(message("q", "e")["yaw"], 0.0)

    def test_arm_and_claw_keys_use_arm_mode(self):
        command = message("i")
        self.assertTrue(command["arm"])
        self.assertEqual(command["az"], 0.5)  # arm up
        self.assertEqual(message("k")["az"], -0.5)
        self.assertEqual(message("l")["ax"], 0.5)  # reach out
        self.assertEqual(message("j")["ax"], -0.5)
        self.assertEqual(message("o")["claw"], 0.5)  # close
        self.assertEqual(message("u", "left shift")["claw"], -1.0)  # open, full speed

    def test_arm_keys_override_walking(self):
        command = message("w", "i")
        self.assertTrue(command["arm"])
        self.assertFalse(command["enable"])  # walking stops while the arm moves

    def test_kneel_align_and_buttons(self):
        self.assertEqual(message("g")["kneel"], 1)  # G kneels down (F is fetch now)
        self.assertEqual(message("t")["kneel"], -1)  # T stands up
        self.assertEqual(message("f")["kneel"], 0)
        self.assertTrue(kb.keys_to_state({"f"})["fetch"])
        self.assertTrue(message("space")["align"])
        state = kb.keys_to_state({"x", "y", "c", "b", "escape", "z"})
        self.assertTrue(all(state[k] for k in ("x", "y", "back", "b", "start", "rt")))
        self.assertFalse(kb.keys_to_state(set())["rt"])


class KeyTrackerTests(unittest.TestCase):
    def test_held_keys_follow_down_and_up(self):
        tracker = kb.KeyTracker()
        tracker.process([("down", "w"), ("down", "left shift")])
        self.assertEqual(tracker.held, {"w", "left shift"})
        tracker.process([("up", "w")])
        self.assertEqual(tracker.held, {"left shift"})

    def test_focus_loss_releases_everything(self):
        tracker = kb.KeyTracker()
        tracker.process([("down", "w"), ("down", "i")])
        tracker.process([("focus_lost",)])
        self.assertEqual(tracker.held, set())

    def test_ctrl_c_in_the_window_quits_instead_of_resetting_the_claw(self):
        for ctrl in ("left ctrl", "right ctrl", "left meta"):
            tracker = kb.KeyTracker()
            tracker.process([("down", ctrl), ("down", "c")])
            state = kb.keys_to_state(tracker.held)
            self.assertTrue(state["start"], ctrl)
            self.assertFalse(state["back"], ctrl)

    def test_focus_is_tracked_for_fetch(self):
        tracker = kb.KeyTracker()
        self.assertTrue(tracker.focused)
        tracker.process([("focus_lost",)])
        self.assertFalse(tracker.focused)
        self.assertFalse(kb.keys_to_state(tracker.held, tracker.focused)["attended"])
        tracker.process([("focus_gained",)])
        self.assertTrue(tracker.focused)

    def test_closing_the_window_quits(self):
        tracker = kb.KeyTracker()
        tracker.process([("quit",)])
        self.assertTrue(kb.keys_to_state(tracker.held)["start"])


class MjpegTests(unittest.TestCase):
    def test_splits_a_stream_into_jpeg_frames(self):
        import io

        part = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n{}\r\n"
        stream = io.BytesIO(b"".join(part.replace(b"{}", str(len(body)).encode(), 1)
                                     .replace(b"{}", body, 1)
                                     for body in (b"\xff\xd8one", b"\xff\xd8second")))
        self.assertEqual(list(kb.mjpeg_frames(stream)), [b"\xff\xd8one", b"\xff\xd8second"])

    def test_truncated_stream_just_ends(self):
        import io

        stream = io.BytesIO(b"--frame\r\nContent-Length: 100\r\n\r\nshort")
        self.assertEqual(list(kb.mjpeg_frames(stream)), [])


class SenderKeyboardTests(unittest.TestCase):
    def test_keyboard_window_shows_video_instead_of_a_browser_tab(self):
        args = sender.build_parser().parse_args(["--keyboard", "--video"])
        self.assertEqual(sender.window_video_url(args), "http://localhost:8090/stream")
        self.assertFalse(sender.open_browser(args))
        self.assertIsNone(sender.window_video_url(sender.build_parser().parse_args(["--keyboard"])))
        self.assertTrue(sender.open_browser(sender.build_parser().parse_args(["--video"])))

    def test_keyboard_flag_disables_the_stick_deadzone(self):
        args = sender.build_parser().parse_args(["--keyboard"])
        self.assertTrue(args.keyboard)
        self.assertEqual(sender.effective_deadzone(args), 0.0)
        self.assertEqual(sender.effective_deadzone(sender.build_parser().parse_args([])), 0.15)


if __name__ == "__main__":
    unittest.main()
