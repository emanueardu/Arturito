# Audio Fix Report

## What was broken
- `arturito.service` (and the `start_arturito.sh` launcher) never set `ROS_DOMAIN_ID`, so the service brought up under the default domain while `robot-web.service` and manual SSH sessions had been using domain 10; that mismatch made `/assistant/say` publishers from SSH invisible to the running stack and introduced intermittent discovery failures.
- `_play_audio` relied on a hand‑rolled `.split()` command, never logged how the CLI playback path was selected, and, because the fallback logic was buried in threading without contextual logs, it was impossible to know when audio was silently dropped.
- The audio player backend choice was opaque to the logs, so diagnosing whether `aplay`, `pyaudio` or another backend was even being used was guesswork.

## Why it failed intermittently
- Without a unified domain ID, ROS 2 publishers and subscribers crossed between systemd and SSH sessions could not see each other unless they happened to initialize after reloading discovery data, producing the “sometimes works” behaviour.
- External playback commands could fail silently if quoting was wrong or if `play_command` was empty, and because the node never reported which backend it was using, silent drops were indistinguishable from successful synthesis.

## What was fixed
- `arturito.service` now exports `ROS_DOMAIN_ID=10` and the launcher script also defaults `ROS_DOMAIN_ID` to 10 before sourcing ROS, guaranteeing that both systemd and interactive launches share the same discovery domain.
- `voice_synth_node` now uses `shlex.split` to format `play_command`, logs every decision path (`play_command` vs internal backend), and warns if the formatted command or fallback backend is empty; failures cascade into the fallback path so nothing is silently ignored.
- `AudioPlayer` exposes its backend name so the node can log which backend is being exercised whenever cached audio is played.

## How to test
1. `sudo systemctl daemon-reload` (read the updated unit file) and `sudo systemctl restart arturito.service`.
2. Tail the voice synth logs to verify the new playback messages: `journalctl -u arturito.service -f | grep -i voice_synth_node`.
3. From the same machine or another SSH session, publish a test utterance: `ros2 topic pub /assistant/say std_msgs/String "{data: 'Prueba de audio desde SSH'}" -1`.
4. Listen for the audible result and confirm the log contains `Reproduciendo ...` for that `/assistant/say` message; if it does, the backend and command flow are healthy.
