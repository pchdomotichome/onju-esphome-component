#pragma once

#include "esphome.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/socket/socket.h"

namespace esphome {
namespace onju {

/**
 * OnjuProtocol — implements the real onju wire protocol:
 * - Upstream: μ-law UDP 16kHz to server:udp_port (chunks of sample_chunk_size)
 * - Downstream: TCP server on tcp_port with binary headers:
 *     0xAA: audio (Opus, header: 0xAA, mic_timeout 16-bit BE, volume, fade, compression_type=2)
 *     0xCC: LED blink (intensity, RGB, fade)
 *     0xDD: stop-listening (hold_s 16-bit BE)
 * - Multicast discovery: responds to 239.0.0.1:12345
 * Hostname and mDNS are handled by ESPHome native components.
 */
class OnjuProtocol : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Configuration setters
  void set_host_name(const std::string& name) { host_name_ = name; }
  void set_server_address(const std::string& addr) { server_address_ = addr; }
  void set_udp_port(uint16_t port) { udp_port_ = port; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_speaker_volume(uint8_t vol) { speaker_volume_ = vol; }
  void set_ptt_mode(bool enabled) { ptt_mode_ = enabled; }
  void set_barge_in_rms(float rms) { barge_in_rms_ = rms; }
  void set_barge_in_holdoff(uint8_t holdoff) { barge_in_holdoff_ = holdoff; }
  void set_barge_in_max_volume(uint8_t max_vol) { barge_in_max_volume_ = max_vol; }
  void set_sample_chunk_size(size_t size) { sample_chunk_size_ = size; }
  void set_mic_gain(int gain) { mic_gain_ = gain; }
  void set_led_count(uint8_t count) { led_count_ = count; }

  // Protocol state
  bool is_playing() const { return is_playing_; }
  bool is_mic_active() const { return mic_active_; }
  bool is_device_enabled() const { return device_enabled_; }
  uint8_t get_speaker_volume() const { return speaker_volume_; }

  // Control methods
  void set_mic_timeout(uint32_t timeout_ms);
  void set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t level, uint8_t fade);
  void set_device_enabled(bool enabled) { device_enabled_ = enabled; }
  void trigger_barge_in();
  void stop_mic();

  // μ-law encode (G.711)
  static uint8_t linear_to_ulaw(int16_t pcm_val);

  // Multicast discovery
  void send_multicast_announce();
  void handle_multicast_query();

 protected:
  // Network (ESPHome socket layer — works on esp-idf and arduino)
  std::unique_ptr<socket::Socket> udp_;
  std::unique_ptr<socket::ListenSocket> tcp_server_;
  std::unique_ptr<socket::Socket> tcp_client_;
  std::string server_ip_str_;

  // Configuration
  std::string host_name_;
  std::string server_address_;
  uint16_t udp_port_ = 3000;
  uint16_t tcp_port_ = 3001;
  uint32_t mic_timeout_ = 30000;
  uint8_t speaker_volume_ = 14;
  bool ptt_mode_ = false;
  float barge_in_rms_ = 3500.0f;
  uint8_t barge_in_holdoff_ = 3;
  uint8_t barge_in_max_volume_ = 14;
  size_t sample_chunk_size_ = 512;
  int mic_gain_ = 2;
  uint8_t led_count_ = 6;

  // State
  bool is_playing_ = false;
  bool mic_active_ = false;
  bool device_enabled_ = true;
  bool barging_in_ = false;
  uint8_t led_r_ = 0, led_g_ = 0, led_b_ = 0;
  uint16_t led_level_ = 0;
  uint8_t led_fade_ = 5;
  unsigned long wakeword_detected_at_ = 0;
  unsigned long server_stopped_speaking_at_ = 0;

  // Buffers
  int16_t* mic_buffer_ = nullptr;
  uint8_t* ulaw_buffer_ = nullptr;

  // μ-law lookup table (G.711)
  static const int16_t exp_lut_[256];

  // TCP header parsing
  enum TcpCommand {
    CMD_AUDIO = 0xAA,
    CMD_LED = 0xCC,
    CMD_STOP = 0xDD
  };

  void handle_tcp_header();
  void handle_audio_command(uint8_t* header);
  void handle_led_command(uint8_t* header);
  void handle_stop_command(uint8_t* header);
  void send_udp_chunk(const uint8_t* data, size_t len);

  // Barge-in detection
  uint8_t barge_in_count_ = 0;
  void check_barge_in(const int16_t* samples, size_t count);

  // mDNS hostname string (informational)
  std::string hostname_;
};

}  // namespace onju
}  // namespace esphome
