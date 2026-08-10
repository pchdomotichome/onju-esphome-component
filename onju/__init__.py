"""Onju protocol component for ESPHome.

Preserves the real onju protocol:
- Upstream: μ-law UDP 16kHz to server:3000
- Downstream: TCP server :3001 with binary headers (0xAA audio, 0xCC LED, 0xDD stop)
- Multicast discovery: 239.0.0.1:12345
- mDNS hostname: onju-{MAC}

All parameters configurable via YAML.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import CONF_ID

AUTO_LOAD = ["audio"]

CODEOWNERS = ["@domus"]

onju_ns = cg.esphome_ns.namespace("onju")
OnjuProtocol = onju_ns.class_("OnjuProtocol", cg.Component)

CONF_ONJU = "onju"
CONF_SERVER_ADDRESS = "server_address"
CONF_UDP_PORT = "udp_port"
CONF_TCP_PORT = "tcp_port"
CONF_MIC_TIMEOUT = "mic_timeout"
CONF_SPEAKER_VOLUME = "speaker_volume"
CONF_PTT_MODE = "ptt_mode"
CONF_BARGE_IN_RMS = "barge_in_rms"
CONF_BARGE_IN_HOLDOFF = "barge_in_holdoff"
CONF_BARGE_IN_MAX_VOLUME = "barge_in_max_volume"
CONF_SAMPLE_CHUNK_SIZE = "sample_chunk_size"
CONF_MIC_GAIN = "mic_gain"
CONF_LED_COUNT = "led_count"
CONF_HOST_NAME = "host_name"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OnjuProtocol),
        cv.Optional(CONF_HOST_NAME, default="onju"): cv.string,
        cv.Required(CONF_SERVER_ADDRESS): cv.string,
        cv.Optional(CONF_UDP_PORT, default=3000): cv.port,
        cv.Optional(CONF_TCP_PORT, default=3001): cv.port,
        cv.Optional(CONF_MIC_TIMEOUT, default=30000): cv.positive_int,
        cv.Optional(CONF_SPEAKER_VOLUME, default=14): cv.int_range(min=0, max=20),
        cv.Optional(CONF_PTT_MODE, default=False): cv.boolean,
        cv.Optional(CONF_BARGE_IN_RMS, default=3500.0): cv.float_,
        cv.Optional(CONF_BARGE_IN_HOLDOFF, default=3): cv.positive_int,
        cv.Optional(CONF_BARGE_IN_MAX_VOLUME, default=14): cv.int_range(min=0, max=20),
        cv.Optional(CONF_SAMPLE_CHUNK_SIZE, default=512): cv.positive_int,
        cv.Optional(CONF_MIC_GAIN, default=2): cv.positive_int,
        cv.Optional(CONF_LED_COUNT, default=6): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_host_name(config[CONF_HOST_NAME]))
    cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS]))
    cg.add(var.set_udp_port(config[CONF_UDP_PORT]))
    cg.add(var.set_tcp_port(config[CONF_TCP_PORT]))
    cg.add(var.set_mic_timeout(config[CONF_MIC_TIMEOUT]))
    cg.add(var.set_speaker_volume(config[CONF_SPEAKER_VOLUME]))
    cg.add(var.set_ptt_mode(config[CONF_PTT_MODE]))
    cg.add(var.set_barge_in_rms(config[CONF_BARGE_IN_RMS]))
    cg.add(var.set_barge_in_holdoff(config[CONF_BARGE_IN_HOLDOFF]))
    cg.add(var.set_barge_in_max_volume(config[CONF_BARGE_IN_MAX_VOLUME]))
    cg.add(var.set_sample_chunk_size(config[CONF_SAMPLE_CHUNK_SIZE]))
    cg.add(var.set_mic_gain(config[CONF_MIC_GAIN]))
    cg.add(var.set_led_count(config[CONF_LED_COUNT]))