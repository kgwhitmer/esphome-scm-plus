# SCM+ Decoder Component for ESPHome

This ESPHome external component provides a SCM+ decoder for ESPHome [remote_receivers](https://esphome.io/components/remote_receiver.html).

## Basic how-to:
1.  Include what is needed for this external component via [packages](https://esphome.io/components/packages.html) directive in your esphome yaml config:
    ```yaml
    packages: 
        scm_plus_receiver: github://kgwhitmer/esphome-scm-plus/scm-plus.yaml
    ```
2.  Add the scm_plus config to your yaml with a remote_reciver id and an on_json message handler...
    ```yaml
    scm_plus_receiver:
      receiver_id: rf_receiver
      on_json:
        then:
          - lambda: |-
              std::string s;
              serializeJson(root, s);
              ESP_LOGD("SCM+ JSON: ", "%s", s.c_str());
    ```

    ...or add the optional meter filtering, adjust frequency, led settings or mqtt settings...

    ```yaml
    scm_plus_receiver:
      id: scm_decoder
      receiver_id: rf_receiver
      led_id: board_status_led
      brightness: 50%
      meter_id: 12345678
      topic: meter-receiver/power-meter
    ```
    ...and matching meter IDs and consumption values are sent via MQTT
