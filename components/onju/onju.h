#pragma once

#include "esphome.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/socket/socket.h"
// FIX #1, #6: includes de audio
#include "esphome/components/microphone/microphone.h"
#include "esphome/components/speaker/speaker.h"

namespace esphome {
namespace onju {

class OnjuProtocol : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ── Setters de configuración (llamados desde to_code) ───────────────
  void set_host_name(const std::string &name)   { host_name_ = name; }
  void set_server_address(const std::string &a) { server_address_ = a; }
  void set_udp_port(uint16_t p)                 { udp_port_ = p; }
  void set_tcp_port(uint16_t p)                 { tcp_port_ = p; }
  void set_speaker_volume(uint8_t v)            { speaker_volume_ = v; }
  void set_ptt_mode(bool e)                     { ptt_mode_ = e; }
  void set_barge_in_rms(float r)                { barge_in_rms_ = r; }
  void set_barge_in_holdoff(uint8_t h)          { barge_in_holdoff_ = h; }
  void set_barge_in_max_volume(uint8_t m)       { barge_in_max_volume_ = m; }
  void set_sample_chunk_size(size_t s)          { sample_chunk_size_ = s; }
  void set_mic_gain(int g)                      { mic_gain_ = g; }
  void set_led_count(uint8_t c)                 { led_count_ = c; }

  // FIX #5: setter de config separado de la lógica de runtime
  void set_default_mic_timeout_ms(uint32_t ms) { default_mic_timeout_ms_ = ms; }

  // FIX #1, #6: setters de audio (llamados desde to_code)
  void set_microphone(microphone::Microphone *mic) { microphone_ = mic; }
  void set_speaker(speaker::Speaker *spk)          { speaker_ = spk; }

  // ── Estado ────────────────────────────────────────────────────────────
  bool is_playing() const         { return is_playing_; }
  bool is_mic_active() const      { return mic_active_; }
  bool is_device_enabled() const  { return device_enabled_; }
  uint8_t get_speaker_volume() const { return speaker_volume_; }

  // ── Control (llamados desde YAML automations o WW callback) ──────────
  // FIX #5: activate_mic() es el método de runtime (no set_mic_timeout)
  void activate_mic(uint32_t timeout_ms);
  void set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t level, uint8_t fade);
  void set_device_enabled(bool e) { device_enabled_ = e; }
  void trigger_barge_in();
  void stop_mic();

  // μ-law encode (G.711)
  static uint8_t linear_to_ulaw(int16_t pcm_val);

  // Multicast discovery
  void send_multicast_announce();
  void handle_multicast_query();

 protected:
  // ── Referencias a componentes ESPHome ─────────────────────────────────
  // FIX #1: miembros de audio que faltaban completamente
  microphone::Microphone *microphone_{nullptr};
  speaker::Speaker       *speaker_{nullptr};

  // ── Sockets ───────────────────────────────────────────────────────────
  std::unique_ptr<socket::Socket> udp_;
  std::unique_ptr<socket::Socket> tcp_server_;   // FIX #11: Socket, no ListenSocket
  std::unique_ptr<socket::Socket> tcp_client_;
  std::string server_ip_str_;

  // ── Configuración ─────────────────────────────────────────────────────
  std::string host_name_;
  std::string server_address_;
  uint16_t udp_port_              = 3000;
  uint16_t tcp_port_              = 3001;
  uint32_t default_mic_timeout_ms_= 5000;  // FIX #5: timeout por defecto, no timestamp
  uint8_t  speaker_volume_        = 14;
  bool     ptt_mode_              = false;
  float    barge_in_rms_          = 3500.0f;
  uint8_t  barge_in_holdoff_      = 3;
  uint8_t  barge_in_max_volume_   = 14;
  size_t   sample_chunk_size_     = 512;
  int      mic_gain_              = 2;
  uint8_t  led_count_             = 6;

  // ── Estado ────────────────────────────────────────────────────────────
  bool     is_playing_            = false;
  bool     mic_active_            = false;   // FIX #5: false en boot
  bool     device_enabled_        = true;
  bool     barging_in_            = false;
  uint32_t mic_timeout_at_ms_     = 0;       // FIX #5: timestamp absoluto, nombre claro
  uint8_t  led_r_ = 0, led_g_ = 0, led_b_ = 0;
  uint16_t led_level_             = 0;
  uint8_t  led_fade_              = 5;
  uint32_t server_stopped_speaking_at_ = 0;

  // ── Buffers de audio ──────────────────────────────────────────────────
  int16_t *mic_buffer_            = nullptr;
  uint8_t *ulaw_buffer_           = nullptr;
  size_t   ulaw_buf_idx_          = 0;       // índice actual en el chunk

  // Downstream: una conexión TCP = header de 6 bytes + payload, y EOF al cerrar.
  // El payload se acumula acá y se le entrega al speaker sólo lo que acepta.
  static constexpr size_t AUDIO_RX_SIZE = 4096;
  uint8_t  audio_rx_[AUDIO_RX_SIZE] = {};
  size_t   audio_rx_len_           = 0;      // bytes pendientes de entregar al speaker
  bool     audio_streaming_        = false;  // leyendo el payload de CMD_AUDIO
  bool     speaker_finish_pending_ = false;  // payload terminado, speaker aún drenando
  uint32_t speaker_empty_since_ms_ = 0;
  uint8_t  compression_type_       = 0;      // 0 = PCM16 crudo, 2 = Opus (no soportado)

  // FIX #3: estado de lectura TCP no bloqueante
  uint8_t  header_buf_[6];
  size_t   header_bytes_read_     = 0;

  // FIX #9: socket UDP separado para discovery multicast
  std::unique_ptr<socket::Socket> udp_discovery_;

  // ── μ-law G.711 lookup table ──────────────────────────────────────────
  static const int16_t exp_lut_[256];

  // ── Comandos TCP ──────────────────────────────────────────────────────
  enum TcpCommand : uint8_t {
    CMD_AUDIO = 0xAA,
    CMD_LED   = 0xCC,
    CMD_STOP  = 0xDD,
  };

  // FIX #3: métodos no bloqueantes
  void poll_tcp_header();            // llamado desde loop(), no bloquea
  void process_complete_header();    // procesa cuando los 6 bytes están listos
  void handle_audio_command();       // usa header_buf_ ya leído
  void handle_led_command();
  void handle_stop_command();

  // FIX #4: consumir el payload PCM no bloqueante, respetando el ring buffer
  void drain_audio_stream();
  void end_audio_stream();

  void send_udp_chunk(const uint8_t *data, size_t len);
  void setup_mic_callback();         // registra add_data_callback en setup()

  // ── Barge-in ──────────────────────────────────────────────────────────
  uint8_t barge_in_count_ = 0;
  void check_barge_in(const int16_t *samples, size_t count);

  // ── Hostname ──────────────────────────────────────────────────────────
  std::string hostname_;
};

}  // namespace onju
}  // namespace esphome
