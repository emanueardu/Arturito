# Audio Stabilization

## Why `aplay` is wrong for Bluetooth
- `aplay` speaks directly to the ALSA device tree (HDMI/analog) and never routes through PipeWire/PulseAudio, so it keeps emitting on HDMI even when the Bluetooth sink is active.
- PipeWire (via `pipewire-pulse`) manages the `bluez_output.*` sink, so desktop-friendly clients like `paplay` land on the Bluetooth speaker automatically.
- Using `paplay` prevents the duplicate path (ALSA + internal playback) that used to trigger repeated or silent output.

## Smoke-test commands
1. `bluetoothctl connect 41:42:FD:4E:3A:32` (pair/activate the BR/EDR sink).
2. `pactl list sinks short` (confirm `bluez_output.41_42_FD_4E_3A_32.1` appears).
3. `paplay /usr/share/sounds/alsa/Front_Center.wav` (ensure PipeWire pushes test tone to the Bluetooth sink).
4. `ros2 topic pub --once /assistant/say std_msgs/msg/String "{data: 'hola'}"` (Drive `voice_synth_node` via the ROS topic and watch for the `paplay` log line with the WAV path).

## Troubleshooting
- If there is no `bluez_output.*` in `pactl list sinks short`, the Bluetooth speaker is not connected/authorized and `paplay` will remain silent.
- When `paplay` fails, check that the Pulse socket exists at `/run/user/1000/pulse/native` and that `XDG_RUNTIME_DIR`/`PULSE_SERVER` are exported (the systemd unit now sets them explicitly).
- PipeWire logs (`journalctl -u arturito.service`) show the sink selection when playback is handed off; silence there means the sink is not running or has been unplugged.
