# Wake Word Assistant (`wake_word_node`)

Este nodo escucha el micrófono, detecta la palabra clave **“robertito”** y, si se reconoce una frase posterior, la envía a la API de Codex/OpenAI. La respuesta se publica en `arturito/say` para que la pipeline TTS (por ejemplo Piper) la reproduzca.

## Dependencias

Instalá los módulos de Python:

```bash
pip3 install sounddevice vosk requests
```

Descargá un modelo de Vosk en español (ejemplo `vosk-model-small-es-0.42`) y colocá la ruta en `~/models/vosk-model-small-es-0.42` o ajustá el parámetro `vosk_model_path`.

## Clave de la API

Define la clave secreta en una variable de entorno **antes de lanzar el nodo**:

```bash
export ROBERTITO_CODEX_API_KEY="TU_CLAVE_SECRETA"
```

Opcionalmente podés configurar:

- `ROBERTITO_CODEX_API_BASE` (por defecto `https://api.openai.com/v1/responses`)
- `ROBERTITO_CODEX_MODEL` (por defecto `gpt-4o-mini`)

## Ejecución

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robertito wake_word_node \
  --ros-args \
  -p vosk_model_path:=/ruta/al/modelo \
  -p wake_word:=robertito
```

Cuando el nodo escuche “robertito”, capturará unos segundos de audio, generará la consulta y publicará la respuesta en `arturito/say`.

