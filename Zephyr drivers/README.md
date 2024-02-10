# Smart-BLE-Remote
keypad drivers
    Drivers
      |________ CMakelists                  C:\ncs\v2.4.2\zephyr\drivers     (add_subdirectory_ifdef(CONFIG_KSCAN_SX1508 sx1508_kscan))
      |________ Kconfig                     C:\ncs\v2.4.2\zephyr\drivers     (source "drivers/sx1508_kscan/Kconfig")
      |________sx1508_kscan
      |         |________ CMakelists           C:\ncs\v2.4.2\zephyr\drivers\sx1508_kscan
      |         |________ Kconfig              C:\ncs\v2.4.2\zephyr\drivers\sx1508_kscan
      |         |________ Kconfig.sx1508       C:\ncs\v2.4.2\zephyr\drivers\sx1508_kscan
      |         |________ kscan_sx1508.c       C:\ncs\v2.4.2\zephyr\drivers\sx1508_kscan
      |
      |________dts                            C:\ncs\v2.4.2\zephyr
      |        |________bindings              C:\ncs\v2.4.2\zephyr\dts\bindings\sx1508_kscan
      |        |________sx1508_kscan          C:\ncs\v2.4.2\zephyr\dts\bindings\sx1508_kscan
      |
      |
      |________include                       C:\ncs\v2.4.2\zephyr
               |________sx1508_kscan         C:\ncs\v2.4.2\zephyr\include\zephyr\drivers\sx1508_kscan
              
              