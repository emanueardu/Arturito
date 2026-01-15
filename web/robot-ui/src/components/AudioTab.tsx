import { useEffect, useRef, useState } from 'react';

const audioUrl = import.meta.env.VITE_AUDIO_URL ?? 'http://localhost:8081/audio';

export default function AudioTab() {
  const audioRef = useRef<HTMLAudioElement>(null);
  const [volume, setVolume] = useState(1);
  const [muted, setMuted] = useState(false);
  const [refreshKey, setRefreshKey] = useState(Date.now());

  useEffect(() => {
    if (audioRef.current) {
      audioRef.current.volume = volume;
    }
  }, [volume]);

  useEffect(() => {
    if (audioRef.current) {
      audioRef.current.muted = muted;
    }
  }, [muted]);

  return (
    <section className="card audio-player">
      <h3>Audio del robot</h3>
      <p>El stream es WAV PCM (mono). Pulsa recargar si el navegador corta la reproducción.</p>
      <audio
        key={refreshKey}
        ref={audioRef}
        controls
        src={`${audioUrl}?cb=${refreshKey}`}
        autoPlay
      />
      <label>
        Volumen
        <input
          type="range"
          min="0"
          max="1"
          step="0.05"
          value={volume}
          onChange={(event) => setVolume(Number(event.target.value))}
        />
      </label>
      <div style={{ display: 'flex', gap: '0.5rem' }}>
        <button className="secondary" type="button" onClick={() => setMuted((prev) => !prev)}>
          {muted ? 'Quitar silencio' : 'Silenciar'}
        </button>
        <button className="secondary" type="button" onClick={() => setRefreshKey(Date.now())}>
          Recargar stream
        </button>
      </div>
      <small>
        Talkback (enviar audio) queda pendiente para una versión segura; por ahora es solo escucha.
      </small>
    </section>
  );
}
