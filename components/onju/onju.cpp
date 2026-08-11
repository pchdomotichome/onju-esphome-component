#include "onju.h"
#include <arpa/inet.h>
#include <array>
#include <cstring>
#include <cmath>
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace onju {

static const char *const TAG = "onju";

// ── G.711 μ-law lookup table (decode) ───────────────────────────────────
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

// ── μ-law encode G.711 ───────────────────────────────────────────────────
uint8_t OnjuProtocol::linear_to_ulaw(int16_t pcm_val) {
  static const int32_t MULAW_BIAS = 0x84;    // 132
  static const int32_t MULAW_CLIP = 32635;   // FIX: era 0x1FFF (8191)

  int32_t sign = (pcm_val < 0) ? 0x80 : 0;
  int32_t sample = pcm_val;
  if (sign) sample = -sample;
  if (sample > MULAW_CLIP) sample = MULAW_CLIP;
  sample += MULAW_BIAS;

  int32_t exponent = 7;
  for (int32_t exp_mask = 0x4000;
       (sample & exp_mask) == 0 && exponent > 0;
       exp_mask >>= 1, exponent--) {}

  int32_t mantissa = (sample >> (exponent + 3)) & 0x0F;
  return static_cast<uint8_t>(~(sign | (exponent << 4) | mantissa));
}

// ── setup() ─────────────────────────────────────────────────────────────
void OnjuProtocol::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Onju protocol");

  // Alocar buffers de audio
  mic_buffer_  = new int16_t[sample_chunk_size_];
  ulaw_buffer_ = new uint8_t[sample_chunk_size_];
  ulaw_buf_idx_ = 0;

  // Construir hostname desde MAC
  uint8_t mac[6];
  get_mac_address_raw(mac);
  char hostname_buf[32];
  snprintf(hostname_buf, sizeof(hostname_buf), "%s-%02X%02X%02X",
           host_name_.c_str(), mac[3], mac[4], mac[5]);
  hostname_ = std::string(hostname_buf);
  ESP_LOGCONFIG(TAG, "Hostname: %s", hostname_.c_str());

  // ── Socket UDP para envío de audio upstream ──────────────────────────
  udp_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (!udp_) {
    this->status_set_error(LOG_STR("Could not create UDP audio socket"));
    this->mark_failed();
    return;
  }
  udp_->setblocking(false);
  int enable = 1;
  udp_->setsockopt(SOL_SOCKET, SO_SNDBUF, &enable, sizeof(int));

  // ── Socket UDP para discovery multicast (puerto 12345) ───────────────
  udp_discovery_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (!udp_discovery_) {
    this->status_set_error(LOG_STR("Could not create UDP discovery socket"));
    this->mark_failed();
    return;
  }
  udp_discovery_->setblocking(false);
  udp_discovery_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  struct sockaddr_in disc_addr;
  memset(&disc_addr, 0, sizeof(disc_addr));
  disc_addr.sin_family      = AF_INET;
  disc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  disc_addr.sin_port        = htons(12345);
  if (udp_discovery_->bind(
        reinterpret_cast<struct sockaddr *>(&disc_addr),
        sizeof(disc_addr)) != 0) {
    ESP_LOGW(TAG, "UDP discovery bind failed (no multicast): %d", errno);
    // No fatal — la parte de audio sigue funcionando
  }

  // ── TCP server para recibir comandos del servidor ────────────────────
  tcp_server_ = socket::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (!tcp_server_) {
    this->status_set_error(LOG_STR("Could not create TCP socket"));
    this->mark_failed();
    return;
  }
  tcp_server_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
  tcp_server_->setblocking(false);
  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family      = AF_INET;
  tcp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  tcp_addr.sin_port        = htons(tcp_port_);
  if (tcp_server_->bind(
        reinterpret_cast<struct sockaddr *>(&tcp_addr),
        sizeof(tcp_addr)) != 0) {
    this->status_set_error(LOG_STR("TCP bind failed"));
    this->mark_failed();
    return;
  }
  tcp_server_->listen(2);
  ESP_LOGCONFIG(TAG, "TCP server on port %d", tcp_port_);

  // FIX VAD: server_address_ configurada es el destino upstream por defecto.
  // Sin esto, el audio del mic solo fluiría mientras haya una conexión TCP
  // abierta (server_ip_str_ se llenaba solo con la IP del peer y se limpiaba
  // al desconectar), y la wake word armaba el mic sin enviar nunca audio.
  if (!server_address_.empty())
    server_ip_str_ = server_address_;

  // FIX #1: registrar callback del micrófono
  setup_mic_callback();

  // Anuncio multicast inicial
  send_multicast_announce();

  // FIX #5: mic_active_ NO se activa aquí — solo cuando el servidor lo pide
  device_enabled_ = true;
  header_bytes_read_ = 0;

  ESP_LOGCONFIG(TAG, "Onju protocol ready (VOX=%s, PTT=%s)",
                YESNO(!ptt_mode_), YESNO(ptt_mode_));
}

// ── setup_mic_callback() — FIX #1: captura real desde I2S ───────────────
void OnjuProtocol::setup_mic_callback() {
  if (!microphone_) {
    ESP_LOGE(TAG, "No microphone configured!");
    return;
  }

  microphone_->add_data_callback([this](const std::vector<uint8_t> &data) {
    // Solo procesar si el mic está activo y hay servidor
    if (!mic_active_ || server_ip_str_.empty()) return;

    // Determinar formato del audio (16-bit o 32-bit según el driver I2S)
    const auto &info = microphone_->get_audio_stream_info();
    const size_t bps = info.get_bits_per_sample() / 8;  // bytes por sample
    if (bps == 0) return;
    const size_t num_samples = data.size() / bps;

    for (size_t i = 0; i < num_samples; i++) {
      // Extraer sample como int16
      int16_t raw;
      if (bps == 4) {
        // SPH0645: 32-bit I2S, datos en los 16 bits altos
        int32_t s32;
        memcpy(&s32, &data[i * 4], 4);
        raw = static_cast<int16_t>(s32 >> 16);
      } else if (bps == 2) {
        memcpy(&raw, &data[i * 2], 2);
      } else {
        continue;
      }

      // Aplicar ganancia con saturación
      int32_t gained = static_cast<int32_t>(raw) * mic_gain_;
      if (gained >  32767) gained =  32767;
      if (gained < -32768) gained = -32768;
      raw = static_cast<int16_t>(gained);

      // Acumular en buffers
      mic_buffer_[ulaw_buf_idx_]  = raw;
      ulaw_buffer_[ulaw_buf_idx_] = linear_to_ulaw(raw);
      ulaw_buf_idx_++;

      // Enviar chunk cuando esté lleno
      if (ulaw_buf_idx_ >= sample_chunk_size_) {
        send_udp_chunk(ulaw_buffer_, ulaw_buf_idx_);
        check_barge_in(mic_buffer_, ulaw_buf_idx_);
        ulaw_buf_idx_ = 0;
      }
    }
  });

  // Iniciar captura — el mic empieza a enviar datos al callback
  microphone_->start();
  ESP_LOGD(TAG, "Mic callback registered and started");
}

// ── loop() ──────────────────────────────────────────────────────────────
void OnjuProtocol::loop() {
  // FIX #3: accept() non-blocking (no usa ready() ni accept_loop_monitored)
  if (tcp_server_) {
    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);
    auto sock = tcp_server_->accept(
        reinterpret_cast<struct sockaddr *>(&source_addr), &addr_len);
    if (sock) {
      if (tcp_client_) {
        tcp_client_->close();
      }
      tcp_client_ = std::move(sock);
      tcp_client_->setblocking(false);

      // Guardar IP del servidor para el UDP upstream
      auto *saddr = reinterpret_cast<struct sockaddr_in *>(&source_addr);
      char ip_buf[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &saddr->sin_addr, ip_buf, sizeof(ip_buf));
      server_ip_str_ = ip_buf;
      header_bytes_read_ = 0;
      opus_bytes_remaining_ = 0;

      ESP_LOGI(TAG, "Servidor conectado: %s", ip_buf);
    }
  }

  // FIX #3 + #4: leer TCP de forma no bloqueante
  if (tcp_client_) {
    // Si hay bytes Opus pendientes de consumir, drenarlos primero
    if (opus_bytes_remaining_ > 0) {
      drain_opus_stream();
    } else {
      // Intentar completar el header de 6 bytes
      poll_tcp_header();
    }
  }

  // FIX #9: discovery en socket separado
  handle_multicast_query();

  // FIX #5: timeout del mic con timestamp correcto
  if (mic_active_ && millis() > mic_timeout_at_ms_) {
    stop_mic();
  }
}

// ── poll_tcp_header() — FIX #3: no bloqueante ───────────────────────────
void OnjuProtocol::poll_tcp_header() {
  if (!tcp_client_) return;

  // Leer bytes disponibles sin bloquear
  ssize_t n = tcp_client_->read(
      header_buf_ + header_bytes_read_,
      6 - header_bytes_read_);

  if (n > 0) {
    header_bytes_read_ += static_cast<size_t>(n);
    if (header_bytes_read_ == 6) {
      process_complete_header();
      header_bytes_read_ = 0;
    }
  } else if (n == 0) {
    // Servidor cerró la conexión
    ESP_LOGW(TAG, "Servidor cerró la conexión TCP");
    tcp_client_->close();
    tcp_client_.reset();
    // No se limpia server_ip_str_: el audio upstream sigue yendo a la IP
    // configurada (server_address_) o a la última IP del servidor conocida.
  } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
    // Error real
    ESP_LOGW(TAG, "TCP read error: %d", errno);
    tcp_client_->close();
    tcp_client_.reset();
  }
  // EAGAIN/EWOULDBLOCK: no hay datos aún, volver en el próximo loop()
}

// ── process_complete_header() ─────────────────────────────────────────────
void OnjuProtocol::process_complete_header() {
  switch (header_buf_[0]) {
    case CMD_AUDIO: handle_audio_command(); break;
    case CMD_LED:   handle_led_command();   break;
    case CMD_STOP:  handle_stop_command();  break;
    default:
      ESP_LOGW(TAG, "Unknown TCP command: 0x%02X", header_buf_[0]);
      // Resetear para intentar re-sincronizar
      break;
  }
}

// ── handle_audio_command() — FIX #2: implementado ────────────────────────
void OnjuProtocol::handle_audio_command() {
  if (!device_enabled_) {
    ESP_LOGW(TAG, "Ignoring audio - device disabled");
    return;
  }

  uint16_t mic_timeout_s    = (header_buf_[1] << 8) | header_buf_[2];
  speaker_volume_           = header_buf_[3];
  uint8_t fade              = header_buf_[4];
  uint8_t compression_type  = header_buf_[5];

  ESP_LOGI(TAG, "CMD_AUDIO: timeout=%ds vol=%d compression=%d",
           mic_timeout_s, speaker_volume_, compression_type);

  set_led(255, 255, 255, 0, fade);
  is_playing_   = true;
  barging_in_   = false;

  // FIX #2: el stream Opus sigue después del header
  // Lo drenamos/decodificamos en drain_opus_stream() llamado desde loop()
  // Por ahora: marcar que hay bytes Opus por consumir
  // El servidor envía frames Opus de tamaño variable hasta que manda CMD_STOP
  // Usamos un valor centinela grande; drain_opus_stream() lee hasta que
  // el servidor no envía más datos o llega el próximo header
  opus_bytes_remaining_ = SIZE_MAX;  // drena hasta CMD_STOP o fin de datos

  // Activar el mic con el timeout indicado por el servidor
  if (mic_timeout_s > 0) {
    activate_mic(static_cast<uint32_t>(mic_timeout_s) * 1000);
  }
}

// ── drain_opus_stream() — FIX #4: consume el stream Opus ─────────────────
void OnjuProtocol::drain_opus_stream() {
  if (!tcp_client_) {
    opus_bytes_remaining_ = 0;
    return;
  }

  // Leer datos disponibles en bloques
  // En una implementación completa, aquí va el decoder Opus → speaker
  // Por ahora: consumir y (si hay speaker) enviar el PCM decodificado
  uint8_t opus_chunk[256];
  ssize_t n = tcp_client_->read(opus_chunk, sizeof(opus_chunk));

  if (n > 0) {
    // TODO: decoder Opus real
    // opus_decode(opus_state_, opus_chunk, n, pcm_out, frame_size, 0);
    // speaker_->write(pcm_out, frame_size * 2);

    // Por ahora: log de bytes recibidos (al menos el buffer se drena)
    ESP_LOGV(TAG, "Opus rx: %d bytes", (int)n);

  } else if (n == 0) {
    // Fin de stream — el servidor cerró
    is_playing_ = false;
    opus_bytes_remaining_ = 0;
  } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
    // No hay más datos por ahora — volver en el próximo loop()
    // opus_bytes_remaining_ se mantiene en SIZE_MAX hasta CMD_STOP
  } else {
    ESP_LOGW(TAG, "Opus stream read error: %d", errno);
    is_playing_ = false;
    opus_bytes_remaining_ = 0;
  }
}

// ── handle_led_command() ──────────────────────────────────────────────────
void OnjuProtocol::handle_led_command() {
  uint8_t intensity = header_buf_[1];
  uint8_t r         = header_buf_[2];
  uint8_t g         = header_buf_[3];
  uint8_t b         = header_buf_[4];
  uint8_t fade      = header_buf_[5];
  set_led(r, g, b, intensity, fade);
  ESP_LOGD(TAG, "CMD_LED: R=%d G=%d B=%d int=%d fade=%d",
           r, g, b, intensity, fade);
}

// ── handle_stop_command() ─────────────────────────────────────────────────
void OnjuProtocol::handle_stop_command() {
  uint16_t hold_s = (header_buf_[1] << 8) | header_buf_[2];
  ESP_LOGI(TAG, "CMD_STOP: hold=%ds", hold_s);
  is_playing_                  = false;
  opus_bytes_remaining_        = 0;   // FIX #4: cancelar drain
  server_stopped_speaking_at_  = millis();
}

// ── activate_mic() — FIX #5: separado del setter de config ───────────────
void OnjuProtocol::activate_mic(uint32_t timeout_ms) {
  mic_timeout_at_ms_ = millis() + timeout_ms;  // timestamp absoluto correcto
  mic_active_        = true;
  ulaw_buf_idx_      = 0;
  ESP_LOGD(TAG, "Mic activado por %d ms", (int)timeout_ms);
}

// ── stop_mic() ────────────────────────────────────────────────────────────
void OnjuProtocol::stop_mic() {
  mic_active_     = false;
  mic_timeout_at_ms_ = 0;
  ulaw_buf_idx_   = 0;
  ESP_LOGD(TAG, "Mic stopped");
}

// ── send_udp_chunk() ─────────────────────────────────────────────────────
void OnjuProtocol::send_udp_chunk(const uint8_t *data, size_t len) {
  if (!udp_ || server_ip_str_.empty()) return;

  struct sockaddr_in saddr;
  memset(&saddr, 0, sizeof(saddr));
  saddr.sin_family = AF_INET;
  saddr.sin_port   = htons(udp_port_);
  if (inet_pton(AF_INET, server_ip_str_.c_str(), &saddr.sin_addr) != 1) {
    ESP_LOGW(TAG, "Invalid server IP: %s", server_ip_str_.c_str());
    return;
  }

  ssize_t sent = udp_->sendto(
      data, len, 0,
      reinterpret_cast<const struct sockaddr *>(&saddr),
      sizeof(saddr));

  if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOMEM) {
    ESP_LOGV(TAG, "UDP sendto: %d", errno);
  }
}

// ── send_multicast_announce() ────────────────────────────────────────────
void OnjuProtocol::send_multicast_announce() {
  if (!udp_discovery_) return;

  std::string announce = hostname_ + " " + ESPHOME_VERSION;
  if (ptt_mode_) announce += " PTT";

  struct sockaddr_in maddr;
  memset(&maddr, 0, sizeof(maddr));
  maddr.sin_family = AF_INET;
  maddr.sin_port   = htons(12345);
  inet_pton(AF_INET, "239.0.0.1", &maddr.sin_addr);

  if (udp_discovery_->sendto(
        announce.c_str(), announce.length(), 0,
        reinterpret_cast<const struct sockaddr *>(&maddr),
        sizeof(maddr)) <= 0) {
    ESP_LOGD(TAG, "Multicast announce failed: %d", errno);
    return;
  }
  ESP_LOGI(TAG, "Multicast announce: %s", announce.c_str());
}

// ── handle_multicast_query() — FIX #9: socket separado ──────────────────
void OnjuProtocol::handle_multicast_query() {
  if (!udp_discovery_) return;

  uint8_t buf[64];
  ssize_t len;
  // Leer solo paquetes del socket de discovery (puerto 12345)
  while ((len = udp_discovery_->read(buf, sizeof(buf))) > 0) {
    send_multicast_announce();
  }
}

// ── check_barge_in() ─────────────────────────────────────────────────────
void OnjuProtocol::check_barge_in(const int16_t *samples, size_t count) {
  if (!is_playing_ || speaker_volume_ > barge_in_max_volume_) {
    barge_in_count_ = 0;
    return;
  }

  float sum = 0;
  for (size_t i = 0; i < count; i++) {
    sum += static_cast<float>(samples[i]) * samples[i];
  }
  float rms = sqrtf(sum / count);

  if (rms > barge_in_rms_) {
    if (++barge_in_count_ >= barge_in_holdoff_) {
      trigger_barge_in();
      barge_in_count_ = 0;
    }
  } else {
    barge_in_count_ = 0;
  }
}

// ── trigger_barge_in() ───────────────────────────────────────────────────
void OnjuProtocol::trigger_barge_in() {
  barging_in_ = true;
  is_playing_ = false;
  opus_bytes_remaining_ = 0;
  ESP_LOGI(TAG, "Barge-in triggered");
}

// ── set_led() ─────────────────────────────────────────────────────────────
void OnjuProtocol::set_led(uint8_t r, uint8_t g, uint8_t b,
                            uint8_t level, uint8_t fade) {
  led_r_ = r; led_g_ = g; led_b_ = b;
  led_level_ = level; led_fade_ = fade;
}

// ── dump_config() ─────────────────────────────────────────────────────────
void OnjuProtocol::dump_config() {
  ESP_LOGCONFIG(TAG, "Onju Protocol:");
  ESP_LOGCONFIG(TAG, "  Hostname: %s", hostname_.c_str());
  ESP_LOGCONFIG(TAG, "  Server: %s", server_address_.c_str());
  ESP_LOGCONFIG(TAG, "  UDP port: %d", udp_port_);
  ESP_LOGCONFIG(TAG, "  TCP port: %d", tcp_port_);
  ESP_LOGCONFIG(TAG, "  Default mic timeout: %d ms", default_mic_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Mic gain: %d", mic_gain_);
  ESP_LOGCONFIG(TAG, "  Sample chunk: %d", (int)sample_chunk_size_);
  ESP_LOGCONFIG(TAG, "  Barge-in RMS: %.0f holdoff: %d maxvol: %d",
                barge_in_rms_, barge_in_holdoff_, barge_in_max_volume_);
  ESP_LOGCONFIG(TAG, "  PTT: %s", YESNO(ptt_mode_));
  ESP_LOGCONFIG(TAG, "  Microphone: %s", microphone_ ? "OK" : "NOT SET");
  ESP_LOGCONFIG(TAG, "  Speaker: %s", speaker_ ? "OK" : "NOT SET");
}

}  // namespace onju
}  // namespace esphome
