# Upstream source

- Repository: https://github.com/Livox-SDK/livox_ros2_driver
- Branch: master
- Imported commit: `1565b976ab1905f911cd2a757fc32433422b701c`
- Imported as source files so a normal project clone includes the driver and local fixes.
- Upstream license: `LICENSE.txt` (individual bundled dependencies retain their notices).

Local changes:

- Configure Mid-70 broadcast code `3GGDJ8U00100391` and enable connection.
- Allow the SDK 2.3.1 bundled fmt C++20 compatibility warning on modern GCC without
  treating that specific warning as an error; other warnings remain unchanged.
