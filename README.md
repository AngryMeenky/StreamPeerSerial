# StreamPeerSerial
A godot extension to support serial port communication.

> Switch to the `module` branch to build as an engine module.

## Usage:

1. Clone and build the plugin.

```bash
git clone -b plugin https://github.com/AngryMeenky/StreamPeerSerial.git --recursive
cd StreamPeerSerial
scons --sconstruct=gdextension_build/SConstruct target=template_debug
```
> The plugin artefacts will be in the `gdextension_build/example/addons/StreamPeerSerial` directory
> after building.

2. The `StreamPeerSerial` class will be added to godot. You can create a new StreamPeerSerial object and set it's 'port', 'baudrate', 'bytesize' and so on. Then `open()` it and communicate with your serial device.

3. There is an example in [serial_port_example](https://github.com/AngryMeenky/serial_port_example/tree/plugin) repo. 

![example](https://raw.githubusercontent.com/AngryMeenky/serial_port_example/main/screen_shot_0.png)
