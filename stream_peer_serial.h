/*************************************************************************/
/*  serial_port.h                                                        */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2023 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2023 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef STREAM_PEER_SERIAL_H
#define STREAM_PEER_SERIAL_H

#ifdef GDEXTENSION
#include <godot_cpp/classes/thread.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/builtin_types.hpp>
#include <godot_cpp/classes/stream_peer_extension.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

using namespace godot;
#else
#include "core/os/thread.h"
#include "core/io/stream_peer.h"
#include "core/string/ustring.h"
#include "core/templates/vector.h"
#include "core/variant/array.h"
#include "core/variant/dictionary.h"
#endif

#include "serial_port_config.h"
#include "libserialport/libserialport.h"

#include <atomic>


class StreamPeerSerial : public StreamPeerExtension {
  GDCLASS(StreamPeerSerial, StreamPeerExtension);

  void _thread_func();

  Thread thread;
  Callable worker;
  String port_name = "";
  sp_port *serial = nullptr;
  sp_event_set *readable = nullptr;
  sp_event_set *writable = nullptr;
  int monitoring_interval = 50; // milliseconds
  std::atomic<bool> fine_working = false;
  std::atomic<bool> monitoring_should_exit = true;


  void _data_received(const PackedByteArray &buf);
  void _process_error(const char *func, sp_return err) const;
  void _defer_error(const char *func, sp_return err) const;

public:
  enum Transport {
    NATIVE    = SP_TRANSPORT_NATIVE,
    USB       = SP_TRANSPORT_USB,
    BLUETOOTH = SP_TRANSPORT_BLUETOOTH,
  };

  StreamPeerSerial();
  ~StreamPeerSerial();

  static Dictionary list_ports();

  static Ref<StreamPeerSerial> open_port(const String &, const Ref<SerialPortConfig> &config);

  Error _get_data(uint8_t *p_buffer, int32_t r_bytes, int32_t *r_received) override;
  Error _get_partial_data(uint8_t *p_buffer, int r_bytes, int32_t *r_received) override;
  Error _put_data(const uint8_t *p_data, int32_t p_bytes, int32_t *r_sent) override;
  Error _put_partial_data(const uint8_t *p_data, int32_t p_bytes, int32_t *r_sent) override;

  int32_t _get_available_bytes() const override;

  void _on_error(const String &where, const String &what);

  Error start_monitoring(uint64_t interval_in_usec = 10000);
  void  stop_monitoring();

  Error  open(String port = "");
  String get_port() const;
  bool   is_open() const;
  void   close();

  bool wait_readable(int64_t ms);
  bool wait_writable(int64_t ms);

  PackedByteArray read_raw(size_t size = 1);
  size_t          write_raw(const PackedByteArray &data);
  Error           flush();

  Error                 apply_config(const Ref<SerialPortConfig> &config);
  Ref<SerialPortConfig> get_config();


  Error   set_baudrate(int64_t baudrate);
  int64_t get_baudrate() const;

  Error   set_data_bits(int64_t bytesize);
  int64_t get_data_bits() const;

  Error                    set_parity(SerialPortConfig::Parity parity);
  SerialPortConfig::Parity get_parity() const;

  Error   set_stop_bits(int64_t stopbits);
  int64_t get_stop_bits() const;

  Error set_flow_control(SerialPortConfig::FlowControl flowcontrol);

  Error set_rts(SerialPortConfig::Rts level = SerialPortConfig::RTS_ON);
  Error set_cts(SerialPortConfig::Cts level = SerialPortConfig::CTS_FLOW);

  Error set_dtr(SerialPortConfig::Dtr level = SerialPortConfig::DTR_ON);
  Error set_dsr(SerialPortConfig::Dsr level = SerialPortConfig::DSR_FLOW);

  bool get_cts();
  bool get_dsr();
  bool get_ri();
  bool get_cd();

protected:
  String _to_string() const;

  static void _bind_methods();
};

VARIANT_ENUM_CAST(StreamPeerSerial::Transport);

#endif // STREAM_PEER_SERIAL_H
