board_runner_args(jlink "--device=nRF52832_xxAA" "--speed=4000")

set(OPENOCD_NRF5_SUBFAMILY "nrf52")
board_runner_args(pyocd "--target=nrf52832" "--frequency=4000000")

include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-nrf5.board.cmake)

# Include your custom LoRa driver & LoRaMAC-node
set(ZEPHYR_LORA_DRIVER_MODULE ${LORA_DRIVER_DIR})
set(ZEPHYR_LORAMAC_NODE_MODULE ${LORAMAC_NODE_DIR})
set(ZEPHYR_LORA_CUSTOM_MODULE ${LORA_CUSTOM_DIR})