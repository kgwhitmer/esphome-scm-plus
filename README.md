# SCM+ Decoder Component for ESPHome

This ESPHome external component provides a SCM+ decoder for ESPHome [remote_receivers](https://esphome.io/components/remote_receiver.html).

## Basic how-to:
1.  Include what is needed for this external component via [packages](https://esphome.io/components/packages.html) directive in your esphome yaml config:
    ```yaml
    packages: 
        scm_plus_receiver: github://kgwhitmer/esphome-scm-plus/scm-plus.yaml
    ```
2.  Add the scm_plusrtl_433 config to your yaml with a remote_reciver id and an on_json_message handler...
    ```yaml
    scm_plus_receiver:
      id: scm_decoder
      receiver_id: rf_receiver
      led_id: board_status_led
      brightness: 50%
      meter_id: 12345678
      topic: meter-receiver/power-meter
    ```
3.  Meter IDs and consumption values are sent via MQTT
