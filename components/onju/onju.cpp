#include "onju.h"
#include <arpa/inet.h>
#include <array>
#include <cstring>
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace onju {

static const char *const TAG = "onju";

// G.711 μ-law exponential lookup table
const int16_t OnjuProtocol::exp_lut_[256] = {
    -32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956,
    -23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764,
    -15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412,
    -11900,-11388,-10876,-10364,-9852,-9340,-8828,-8316,
    -7932,-7676,-7420,-7164,-6908,-6652,-6396,-6140,
    -5884,-5628,-5372,-5116,-4860,-4604,-4348,-4092,
    -3900,-3772,-3644,-3516,-3388,-3260,-3132,-3004,
    -2876,-2748,-2620,-2492,-2364,-2236,-2108,-1980,
    -1884,-1820,-1756,-1692,-1628,-1564,-1500,-1436,
    -1372,-1308,-1244,-1180,-1116,-1052,-988,-924,
    -876,-844,-812,-780,-748,-716,-684,-652,
    -620,-588,-556,-524,-492,-460,-428,-396,
    -372,-356,-340,-324,-308,-292,-276,-260,
    -244,-228,-212,-196,-180,-164,-148,-132,
    -120,-112,-104,-96,-88,-80,-72,-64,
    -56,-48,-40,-32,-24,-16,-8,0,
    32124,31100,30076,29052,28028,27004,25980,24956,
    23932,22908,21884,20860,19836,18812,17788,16764,
    15996,15484,14972,14460,13948,13436,12924,12412,
    11900,11388,10876,10364,9852,9340,8828,8316,
    7932,7676,7420,7164,6908,6652,6396,6140,
    5884,5628,5372,5116,4860,4604,4348,4092,
    3900,3772,3644,3516,3388,3260,3132,3004,
    2876,2748,2620,2492,2364,2236,2108,1980,
    1884,1820,1756,1692,1628,1564,1500,1436,
    1372,1308,1244,1180,1116,1052,988,924,
    876,844,812,780,748,716,684,652,
    620,588,556,524,492,460,428,396,
    372,356,340,324,308,292,276,260,
    244,228,212,196,180,164,148,132,
    120,112,104,96,88,80,72,64,
    56,48,40,32,24,16,8,0
};

uint8_t OnjuProtocol::linear_to_ulaw(int16_t pcm_val) {
  const int16_t MULAW_BIAS = 0x84;
  const int16_t MULAW_MAX = 0x1FFF;

  int16_t sign = (pcm_val < 0) ? 0x80 : 0;
  if (sign) pcm_val = -pcm_val;
  if (pcm_val > MULAW_MAX) pcm_val = MULAW_MAX;

  pcm_val += MULAW_BIAS;
  int16_t exponent = 7;

  for (int16_t exp_mask = 0x4000; (pcm_val & exp_mask) == 0 && exponent > 0; exp_mask >>= 1, exponent--);

  int16_t mantissa = (pcm_val >> (exponent + 3)) & 0x0F;
  uint8_t ulaw_byte = ~(sign | (exponent << 4) | mantissa);
  return ulaw_byte;
}

void OnjuProtocol::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Onju protocol");

  // Allocate buffers
  mic_buffer_ = new int16_t[sample_chunk_size_];
  ulaw_buffer_ = new uint8_t[sample_chunk_size_];

  // Build hostname from prefix + last 3 bytes of MAC (informational; ESPHome owns the real hostname)
  uint8_t mac[6];
  get_mac_address_raw(mac);
  char hostname_buf[32];
  snprintf(hostname_buf, sizeof(hostname_buf), "%s-%02X%02X%02X",
           host_name_.c_str(), mac[3], mac[4], mac[5]);
  hostname_ = std::string(hostname_buf);
  ESP_LOGCONFIG(TAG, "Hostname: %s", hostname_.c_str());

  // Start UDP listener (multicast discovery + mic upstream target)
  udp_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (udp_ == nullptr) {
    this->status_set_error(LOG_STR("Could not create UDP socket"));
    this->mark_failed();
    return;
  }
  udp_->setblocking(false);
  int enable = 1;
  udp_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  udp_->setsockopt(SOL_SOCKET, SO_BROADCAST, &enable, sizeof(int));
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(udp_port_);
  if (udp_->bind((struct sockaddr *) &addr, sizeof(addr)) != 0) {
    this->status_set_error(LOG_STR("UDP bind failed"));
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "UDP listener on port %d", udp_port_);

  // Start TCP server
  tcp_server_ = socket::socket_listen_loop_monitored(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_ == nullptr) {
    this->status_set_error(LOG_STR("Could not create TCP socket"));
    this->mark_failed();
    return;
  }
  tcp_server_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  tcp_server_->setblocking(false);
  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  tcp_addr.sin_port = htons(tcp_port_);
  if (tcp_server_->bind((struct sockaddr *) &tcp_addr, sizeof(tcp_addr)) != 0) {
    this->status_set_error(LOG_STR("TCP bind failed"));
    this->mark_failed();
    return;
  }
  tcp_server_->listen(4);
  ESP_LOGCONFIG(TAG, "TCP server on port %d", tcp_port_);

  // Send initial multicast announcement
  send_multicast_announce();

  // Device starts enabled
  device_enabled_ = true;
  if (ptt_mode_) {
    ESP_LOGCONFIG(TAG, "PTT mode enabled");
  } else {
    ESP_LOGCONFIG(TAG, "VOX mode enabled");
  }
}

void OnjuProtocol::loop() {
  // Accept incoming TCP connections
  if (tcp_server_ && tcp_server_->ready()) {
    while (true) {
      struct sockaddr_storage source_addr;
      socklen_t addr_len = sizeof(source_addr);
      auto sock = tcp_server_->accept_loop_monitored((struct sockaddr *) &source_addr, &addr_len);
      if (!sock)
        break;
      // Replace existing client with the newest connection
      if (tcp_client_ != nullptr) {
        tcp_client_->close();
      }
      tcp_client_ = std::move(sock);
      char peername[socket::SOCKADDR_STR_LEN];
      tcp_client_->getpeername_to(peername);
      ESP_LOGI(TAG, "New TCP client: %s", peername);
      // Save server IP for UDP upstream
      struct sockaddr_in *saddr = reinterpret_cast<struct sockaddr_in *>(&source_addr);
      char ip_buf[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &saddr->sin_addr, ip_buf, sizeof(ip_buf));
      server_ip_str_ = ip_buf;
    }
  }

  // Handle data from connected client
  if (tcp_client_ != nullptr && tcp_client_->ready()) {
    handle_tcp_header();
  }

  // Handle multicast discovery
  handle_multicast_query();

  // Check mic timeout
  if (mic_active_ && millis() > mic_timeout_) {
    stop_mic();
  }
}

void OnjuProtocol::dump_config() {
  ESP_LOGCONFIG(TAG, "Onju protocol:");
  ESP_LOGCONFIG(TAG, "  Host name: %s", host_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Server address: %s", server_address_.c_str());
  ESP_LOGCONFIG(TAG, "  UDP port: %u", udp_port_);
  ESP_LOGCONFIG(TAG, "  TCP port: %u", tcp_port_);
  ESP_LOGCONFIG(TAG, "  Mic timeout: %u", mic_timeout_);
  ESP_LOGCONFIG(TAG, "  Speaker volume: %u", speaker_volume_);
  ESP_LOGCONFIG(TAG, "  PTT mode: %s", YESNO(ptt_mode_));
  ESP_LOGCONFIG(TAG, "  Barge-in RMS: %.1f", barge_in_rms_);
  ESP_LOGCONFIG(TAG, "  Barge-in holdoff: %u", barge_in_holdoff_);
  ESP_LOGCONFIG(TAG, "  Barge-in max volume: %u", barge_in_max_volume_);
  ESP_LOGCONFIG(TAG, "  Sample chunk size: %zu", sample_chunk_size_);
  ESP_LOGCONFIG(TAG, "  Mic gain: %d", mic_gain_);
  ESP_LOGCONFIG(TAG, "  LED count: %u", led_count_);
}

void OnjuProtocol::set_mic_timeout(uint32_t timeout_ms) {
  mic_timeout_ = millis() + timeout_ms;
  mic_active_ = true;
}

void OnjuProtocol::set_led(uint8_t r, uint8_t g, uint8_t b, uint8_t level, uint8_t fade) {
  led_r_ = r;
  led_g_ = g;
  led_b_ = b;
  led_level_ = level;
  led_fade_ = fade;
}

void OnjuProtocol::trigger_barge_in() {
  barging_in_ = true;
  ESP_LOGI(TAG, "Barge-in triggered");
}

void OnjuProtocol::stop_mic() {
  mic_active_ = false;
  mic_timeout_ = 0;
  ESP_LOGD(TAG, "Mic stopped");
}

void OnjuProtocol::send_multicast_announce() {
  if (udp_ == nullptr)
    return;

  std::string announce = hostname_ + " " + ESPHOME_VERSION;
  if (ptt_mode_) {
    announce += " PTT";
  }

  struct sockaddr_storage maddr;
  socklen_t mlen = socket::set_sockaddr((struct sockaddr *) &maddr, sizeof(maddr), "239.0.0.1", 12345);
  if (udp_->sendto(announce.c_str(), announce.length(), 0, (const struct sockaddr *) &maddr, mlen) <= 0) {
    ESP_LOGD(TAG, "Multicast announce sendto error %d", errno);
    return;
  }

  ESP_LOGI(TAG, "Multicast announce sent: %s", announce.c_str());
}

void OnjuProtocol::handle_multicast_query() {
  if (udp_ == nullptr)
    return;

  std::array<uint8_t, 64> buf;
  ssize_t len;
  while ((len = udp_->read(buf.data(), buf.size())) > 0) {
    // Respond to discovery
    send_multicast_announce();
  }
}

void OnjuProtocol::handle_tcp_header() {
  if (!tcp_client_) return;

  // Wait for 6-byte header (500ms timeout)
  uint32_t wait_start = millis();
  uint8_t header[6];
  size_t got = 0;
  while (got < 6) {
    ssize_t n = tcp_client_->read(header + got, 6 - got);
    if (n > 0) {
      got += n;
    } else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
      ESP_LOGW(TAG, "TCP client read error %d", errno);
      tcp_client_->close();
      tcp_client_.reset();
      return;
    }
    if ((millis() - wait_start) > 500) {
      return;
    }
    yield();
  }

  if (header[0] == CMD_AUDIO) {
    handle_audio_command(header);
  } else if (header[0] == CMD_LED) {
    handle_led_command(header);
  } else if (header[0] == CMD_STOP) {
    handle_stop_command(header);
  }
}

void OnjuProtocol::handle_audio_command(uint8_t* header) {
  if (!device_enabled_) {
    ESP_LOGW(TAG, "Ignoring audio - device disabled");
    return;
  }

  uint16_t timeout = (header[1] << 8) | header[2];
  speaker_volume_ = header[3];
  uint8_t fade = header[4];
  uint8_t compression_type = header[5];

  ESP_LOGI(TAG, "Audio received (compression=%d), timeout=%ds, volume=%d",
           compression_type, timeout, speaker_volume_);

  set_led(255, 255, 255, 0, fade);
  is_playing_ = true;
  barging_in_ = false;

  // TODO: Implement audio playback (Opus decode → I2S speaker)
  // For now, just consume the stream
}

void OnjuProtocol::handle_led_command(uint8_t* header) {
  uint8_t intensity = header[1];
  uint8_t r = header[2];
  uint8_t g = header[3];
  uint8_t b = header[4];
  uint8_t fade = header[5];

  set_led(r, g, b, intensity, fade);
  ESP_LOGD(TAG, "LED: R=%d G=%d B=%d intensity=%d fade=%d", r, g, b, intensity, fade);
}

void OnjuProtocol::handle_stop_command(uint8_t* header) {
  uint16_t hold_s = (header[1] << 8) | header[2];
  ESP_LOGI(TAG, "Stop listening, hold=%ds", hold_s);

  is_playing_ = false;
  server_stopped_speaking_at_ = millis();
}

void OnjuProtocol::send_udp_chunk(const uint8_t* data, size_t len) {
  if (udp_ == nullptr || server_ip_str_.empty()) {
    return;
  }

  struct sockaddr_storage saddr;
  socklen_t sl = socket::set_sockaddr((struct sockaddr *) &saddr, sizeof(saddr), server_ip_str_, udp_port_);
  if (sl == 0) {
    ESP_LOGW(TAG, "Invalid server IP: %s", server_ip_str_.c_str());
    return;
  }
  if (udp_->sendto(data, len, 0, (const struct sockaddr *) &saddr, sl) <= 0) {
    ESP_LOGW(TAG, "UDP sendto error %d", errno);
  }
}

void OnjuProtocol::check_barge_in(const int16_t* samples, size_t count) {
  if (!is_playing_ || speaker_volume_ > barge_in_max_volume_) {
    barge_in_count_ = 0;
    return;
  }

  // Calculate RMS
  float sum = 0;
  for (size_t i = 0; i < count; i++) {
    sum += samples[i] * samples[i];
  }
  float rms = sqrt(sum / count);

  if (rms > barge_in_rms_) {
    barge_in_count_++;
    if (barge_in_count_ >= barge_in_holdoff_) {
      trigger_barge_in();
      barge_in_count_ = 0;
    }
  } else {
    barge_in_count_ = 0;
  }
}

}  // namespace onju
}  // namespace esphome