# OmniSensContact

Software for a Window contact based on ESP32 C6 communicating via Zigbee.

in sdkconfig muss CONFIG\_FREERTOS\_HZ von 100 auf 1000 Hz erhöt werden! sonst compile fehler

um neues konfig file zu laden zumbeischpiel um fensterkontakt hinzuzufügen, kompletten speicher überschreiben und dann config und code laden, um sicherzustellen das zigbee neu verbinden und konfigurieren muss.

