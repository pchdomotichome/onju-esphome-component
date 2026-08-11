"""Onju protocol component for ESPHome.

Preserves the real onju wire protocol:
- Upstream: μ-law UDP 16kHz to server:3000
- Downstream: TCP server :3001 with binary headers
    0xAA: audio (mic_timeout 16-bit BE, volume, fade, compression_type)
    0xCC: LED blink (intensity, R, G, B, fade)
    0xDD: stop-listening (hold_s 16-bit BE)
- Multicast discovery: 239.0.0.1:12345
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import microphone, speaker
from esphome.const import CONF_ID

AUTO_LOAD = []
DEPENDENCIES = ["network", "wifi"]

CODEOWNERS = ["@domus"]

onju_ns = cg.esphome_ns.namespace("onju")
OnjuProtocol = onju_ns.class_("OnjuProtocol", cg.Component)

CONF_ONJU                 = "onju"
CONF_SERVER_ADDRESS       = "server_address"
CONF_UDP_PORT             = "udp_port"
CONF_TCP_PORT             = "tcp_port"
CONF_DEFAULT_MIC_TIMEOUT  = "default_mic_timeout_ms"
CONF_SPEAKER_VOLUME       = "speaker_volume"
CONF_PTT_MODE             = "ptt_mode"
CONF_BARGE_IN_RMS         = "barge_in_rms"
CONF_BARGE_IN_HOLDOFF     = "barge_in_holdoff"
CONF_BARGE_IN_MAX_VOLUME  = "barge_in_max_volume"
CONF_SAMPLE_CHUNK_SIZE    = "sample_chunk_size"
CONF_MIC_GAIN             = "mic_gain"
CONF_LED_COUNT            = "led_count"
CONF_HOST_NAME            = "host_name"
CONF_MICROPHONE           = "microphone"
CONF_SPEAKER              = "speaker"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OnjuProtocol),

        cv.Required(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
        cv.Required(CONF_SPEAKER):    cv.use_id(speaker.Speaker),

        cv.Optional(CONF_HOST_NAME, default="onju"): cv.string,
        cv.Required(CONF_SERVER_ADDRESS): cv.string,
        cv.Optional(CONF_UDP_PORT,            default=3000):   cv.port,
        cv.Optional(CONF_TCP_PORT,            default=3001):   cv.port,
        cv.Optional(CONF_DEFAULT_MIC_TIMEOUT, default=5000):   cv.positive_int,
        cv.Optional(CONF_SPEAKER_VOLUME,      default=14):     cv.int_range(min=0, max=20),
        cv.Optional(CONF_PTT_MODE,            default=False):  cv.boolean,
        cv.Optional(CONF_BARGE_IN_RMS,        default=3500.0): cv.float_,
        cv.Optional(CONF_BARGE_IN_HOLDOFF,    default=3):      cv.positive_int,
        cv.Optional(CONF_BARGE_IN_MAX_VOLUME, default=14):     cv.int_range(min=0, max=20),
        cv.Optional(CONF_SAMPLE_CHUNK_SIZE,   default=512):    cv.positive_int,
        cv.Optional(CONF_MIC_GAIN,            default=2):      cv.positive_int,
        cv.Optional(CONF_LED_COUNT,           default=6):      cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    cg.add(var.set_microphone(mic))

    spk = await cg.get_variable(config[CONF_SPEAKER])
    cg.add(var.set_speaker(spk))

    cg.add(var.set_host_name(config[CONF_HOST_NAME]))
    cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS]))
    cg.add(var.set_udp_port(config[CONF_UDP_PORT]))
    cg.add(var.set_tcp_port(config[CONF_TCP_PORT]))
    cg.add(var.set_default_mic_timeout_ms(config[CONF_DEFAULT_MIC_TIMEOUT]))
    cg.add(var.set_speaker_volume(config[CONF_SPEAKER_VOLUME]))
    cg.add(var.set_ptt_mode(config[CONF_PTT_MODE]))
    cg.add(var.set_barge_in_rms(config[CONF_BARGE_IN_RMS]))
    cg.add(var.set_barge_in_holdoff(config[CONF_BARGE_IN_HOLDOFF]))
    cg.add(var.set_barge_in_max_volume(config[CONF_BARGE_IN_MAX_VOLUME]))
    cg.add(var.set_sample_chunk_size(config[CONF_SAMPLE_CHUNK_SIZE]))
    cg.add(var.set_mic_gain(config[CONF_MIC_GAIN]))
    cg.add(var.set_led_count(config[CONF_LED_COUNT]))
