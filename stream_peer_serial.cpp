/*************************************************************************/
/*  serial_port.cpp                                                      */
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

#include "stream_peer_serial.h"

#ifdef GDEXTENSION
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;
#else
#include "core/object/class_db.h"
#include "core/os/memory.h"
#include "core/os/os.h"
#endif
#include <string>


void StreamPeerSerial::_data_received(const PackedByteArray &buf) {
  emit_signal("data_received", buf);
}


StreamPeerSerial::StreamPeerSerial():
  thread() {
}


StreamPeerSerial::~StreamPeerSerial() {
  stop_monitoring();
  close();
  if(serial != nullptr) {
    sp_free_port(serial);
    serial = nullptr;
  }
}


Dictionary StreamPeerSerial::list_ports() {
  sp_port **ports = nullptr;
  Dictionary info_dict;
  if(sp_list_ports(&ports) == SP_OK) {
    for(int idx = 0; ports[idx]; ++idx) {
      Dictionary info;
      sp_port *port = ports[idx];
      info["desc"] = sp_get_port_description(port);
      switch(sp_get_port_transport(port)) {
        case SP_TRANSPORT_NATIVE:
          info["transport"] = "native";
          info["hw_id"] = "n/a";
        break;
        case SP_TRANSPORT_USB:
          info["transport"]    = "USB";
          info["manufacturer"] = sp_get_port_usb_manufacturer(port);
          info["product"] = sp_get_port_usb_product(port);
          info["serial_number"] = sp_get_port_usb_serial(port);
          if(int bus, addr; sp_get_port_usb_bus_address(port, &bus, &addr) == SP_OK) {
            info["bus"] = bus;
            info["address"] = addr;
          }
          if(int vid, pid; sp_get_port_usb_vid_pid(port, &vid, &pid) == SP_OK) {
            char id[16];
            snprintf(&id[0], sizeof(id) - 1, "%04X:%04X", vid, pid);
            info["vendor_id"] = vid;
            info["product_id"] = pid;
            info["hw_id"] = &id[0];
          }
          else {
            info["hw_id"] = "n/a";
          }
        break;
        case SP_TRANSPORT_BLUETOOTH:
          info["transport"] = "Bluetooth";
          info["address"] = sp_get_port_bluetooth_address(port);
          info["hw_id"] = info["address"];
        break;
      }

      //UtilityFunctions::print("Found port: ", info);
      info_dict[sp_get_port_name(port)] = Variant(info);
    }

    sp_free_port_list(ports);
  }

  return info_dict;
}


Ref<StreamPeerSerial> StreamPeerSerial::open_port(const String &name, const Ref<SerialPortConfig> &conf) {
  Ref<StreamPeerSerial> sps;
  sps.instantiate();

  if(sps->open(name) == OK) {
    if(conf.is_valid() && sps->apply_config(conf) != OK) {
      sps->close();
      sps.unref();
    }
  }
  else {
    sps.unref();
  }

  return sps;
}


void StreamPeerSerial::_on_error(const String &where, const String &what) {
  fine_working.store(false);
  emit_signal("got_error", where, what);
}


Error StreamPeerSerial::start_monitoring(uint64_t interval_in_msec) {
  ERR_FAIL_COND_V_MSG(!monitoring_should_exit, ERR_ALREADY_IN_USE, "Monitor already started.");
  //UtilityFunctions::print("Monitoring starting: ", interval_in_msec);
  stop_monitoring();
  monitoring_should_exit = false;
  monitoring_interval = interval_in_msec;
  if(!worker.is_valid()) {
    worker = create_custom_callable_function_pointer(this, &StreamPeerSerial::_thread_func);
  }
  thread.start(worker);

  return OK;
}


void StreamPeerSerial::stop_monitoring() {
  //UtilityFunctions::print("Monitoring stoping");
  monitoring_should_exit = true;
  if(thread.is_started()) {
    thread.wait_to_finish();
  }
}


void StreamPeerSerial::_thread_func() {
  // set up for event monitoring
  sp_event_set *events = nullptr;
  if(sp_new_event_set(&events) == SP_OK) {
    constexpr auto event_bits = static_cast<sp_event>(SP_EVENT_RX_READY | SP_EVENT_ERROR);
    if(sp_add_port_events(events, serial, event_bits) != SP_OK) {
      sp_free_event_set(events);
      events = nullptr;
    }
  }

  while(!monitoring_should_exit.load()) {
    if(int avail; fine_working.load() && (avail = _get_available_bytes()) > 0) {
      // data is available to read
      call_deferred("_data_received", read_raw(avail));
    }
    else if(avail < 0) {
      // port is in an error state
      break; // don't perform the monitoring any longer
    }
    else if(events != nullptr) {
      // wait for an event to occur
      if(sp_wait(events, monitoring_interval) != SP_OK) {
        goto delay;
      }
    }
    else {
delay:
      // just do a simple sleep if all else fails
      OS::get_singleton()->delay_msec(monitoring_interval);
    }
  }

  if(events != nullptr) {
    sp_free_event_set(events);
    events = nullptr;
  }
}


Error StreamPeerSerial::open(String name) {
  // only attempt to open the port if the string isn't empty
  if(!name.is_empty()) {
    sp_port *port = nullptr;
    if(auto err = sp_get_port_by_name(name.utf8().get_data(), &port); err != SP_OK) {
      _process_error(__FUNCTION__, err);
    }

    if(port == nullptr) {
      return ERR_CANT_RESOLVE; // couldn't find or couldn't copy port
    }

    if(auto err = sp_open(port, SP_MODE_READ_WRITE); err != SP_OK) {
      _process_error(__FUNCTION__, err);
      sp_free_port(port);
      return ERR_CANT_OPEN; // can't open the actual serial port
    }

    // replace any previously opened serial port
    close();
    if(sp_new_event_set(&readable) == SP_OK) {
      constexpr auto events = static_cast<sp_event>(SP_EVENT_RX_READY | SP_EVENT_ERROR);
      if(sp_add_port_events(readable, port, events) != SP_OK) {
        sp_free_event_set(readable);
        readable = nullptr;
      }
    }

    if(sp_new_event_set(&writable) == SP_OK) {
      if(sp_add_port_events(writable, port, SP_EVENT_TX_READY) != SP_OK) {
        sp_free_event_set(writable);
        writable = nullptr;
      }
    }

    serial = port;
    port_name = name;
    fine_working.store(true);
    emit_signal("opened", name);
    return OK;
  }

  return ERR_PARAMETER_RANGE_ERROR;
}


bool StreamPeerSerial::is_open() const {
  return serial != nullptr;
}


void StreamPeerSerial::_process_error(const char *func, sp_return err) const {
  switch(err) {
    case SP_ERR_ARG:
      const_cast<StreamPeerSerial *>(this)->_on_error(func, "Invalid Argument");
    break;

    case SP_ERR_FAIL: {
      auto msg = sp_last_error_message();
      const_cast<StreamPeerSerial *>(this)->_on_error(func, msg);
      sp_free_error_message(msg);
    } break;

    case SP_ERR_MEM:
      const_cast<StreamPeerSerial *>(this)->_on_error(func, "Allocation Failure");
    break;

    case SP_ERR_SUPP:
      const_cast<StreamPeerSerial *>(this)->_on_error(func, "Operation Not Supported");
    break;
  }
}


void StreamPeerSerial::_defer_error(const char *func, sp_return err) const {
  switch(err) {
    case SP_ERR_ARG:
      const_cast<StreamPeerSerial *>(this)->call_deferred(
          "_on_error", String(func), String("Invalid Argument")
      );
    break;

    case SP_ERR_FAIL: {
      auto msg = sp_last_error_message();
      const_cast<StreamPeerSerial *>(this)->call_deferred("_on_error", String(func), String(msg));
      sp_free_error_message(msg);
    } break;

    case SP_ERR_MEM:
      const_cast<StreamPeerSerial *>(this)->call_deferred(
          "_on_error", String(func), String("Allocation Failure")
      );
    break;

    case SP_ERR_SUPP:
      const_cast<StreamPeerSerial *>(this)->call_deferred(
          "_on_error", String(func), String("Operation Not Supported")
      );
    break;
  }
}


void StreamPeerSerial::close() {
  if(serial) {
    if(auto err = sp_close(serial); err != SP_OK) {
      _process_error(__FUNCTION__, err);
    }

    sp_free_event_set(readable);
    sp_free_event_set(writable);
    sp_free_port(serial);
    readable = nullptr;
    writable = nullptr;
    serial = nullptr;

    emit_signal("closed", port_name);
    port_name = "";
  }

  fine_working.store(false);
}


int32_t StreamPeerSerial::_get_available_bytes() const {
  if(serial != nullptr) {
    if(auto retVal = sp_input_waiting(serial); retVal < 0) {
      _defer_error(__FUNCTION__, retVal);
      UtilityFunctions::printerr("Can't determine available bytes for: ", port_name);
    }
    else {
      //UtilityFunctions::print(port_name, " -> ", retVal);
      return retVal;
    }
  }

  return 0;
}


bool StreamPeerSerial::wait_readable(int64_t ms) {
  if(auto err = sp_wait(readable, ms < 0 ? 0 : ms); err != SP_OK) {
    _defer_error(__FUNCTION__, err);
    return false;
  }

  return true;
}


bool StreamPeerSerial::wait_writable(int64_t ms) {
  if(auto err = sp_wait(writable, ms < 0 ? 0 : ms); err != SP_OK) {
    _defer_error(__FUNCTION__, err);
    return false;
  }

  return true;
}


PackedByteArray StreamPeerSerial::read_raw(size_t size) {
  PackedByteArray raw;
  raw.resize(size);

  int32_t bytes_read = 0;
  int32_t request = static_cast<int32_t>(size);
  auto err = _get_data(raw.ptrw(), static_cast<int32_t>(size), &bytes_read);
  if(bytes_read < request) {
    raw.resize(bytes_read);
  }

  return raw;
}


Error StreamPeerSerial::_get_data(uint8_t *p_buffer, int32_t bytes, int32_t *r_received) {
  if(bytes < 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  Error result = OK;
  sp_return err = sp_blocking_read(serial, p_buffer, bytes, 0);
  if(err < 0) {
    _defer_error(__FUNCTION__, err);
    result = ERR_FILE_CANT_READ;
  }

  *r_received = std::max(static_cast<int32_t>(err), 0);

  //UtilityFunctions::print(port_name, "._get_data(", bytes, ") -> ", *r_received);
  return result;
}


Error StreamPeerSerial::_get_partial_data(uint8_t *p_buffer, int32_t bytes, int32_t *r_received) {
  auto avail = _get_available_bytes();
  if(avail <= 0) {
    return ERR_BUSY;
  }
  else if(avail < bytes) {
    bytes = avail;
  }

  return _get_data(p_buffer, bytes, r_received);
}


size_t StreamPeerSerial::write_raw(const PackedByteArray &data) {
  int32_t sent = 0;
  (void) _put_data(data.ptr(), static_cast<int32_t>(data.size()), &sent);
  //UtilityFunctions::print(port_name, ".write_raw(", data.size(), ") -> ", sent);
  return sent;
}


Error StreamPeerSerial::_put_data(const uint8_t *p_data, int32_t bytes, int32_t *r_sent) {
  if(bytes < 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  Error result = OK;
  auto err = sp_blocking_write(serial, p_data, bytes, 0);
  if(err < 0) {
    _defer_error(__FUNCTION__, err);
    result = ERR_FILE_CANT_WRITE;
  }
  else {
    *r_sent = err;
  }

  //UtilityFunctions::print(port_name, "._put_data(", bytes, ") -> ", *r_sent);
  return result;
}


Error StreamPeerSerial::_put_partial_data(const uint8_t *p_data, int32_t bytes, int32_t *r_sent) {
  if(bytes < 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  Error result = OK;
  auto err = sp_nonblocking_write(serial, p_data, bytes);
  if(err < 0) {
    _defer_error(__FUNCTION__, err);
    result = ERR_FILE_CANT_WRITE;
  }
  else {
    *r_sent = err;
  }

  //UtilityFunctions::print(port_name, "._put_partial_data(", bytes, ") -> ", *r_sent);
  return result;
}


String StreamPeerSerial::get_port() const {
  return port_name;
}


Error StreamPeerSerial::set_baudrate(int64_t baudrate) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_baudrate(serial, static_cast<uint32_t>(baudrate));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }
 
  return OK;
}


int64_t StreamPeerSerial::get_baudrate() const {
  sp_port_config *config = nullptr;
  int64_t baud = 0;

  if(serial != nullptr) {
    if(auto err = sp_new_config(&config); err < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if((err = sp_get_config(serial, config)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int rate = 0; (err = sp_get_config_baudrate(config, &rate)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else {
      baud = rate;
    }

    if(config != nullptr) {
      sp_free_config(config);
      config = nullptr;
    }
  }

  return baud;
}


Error StreamPeerSerial::set_data_bits(int64_t bits) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_bits(serial, static_cast<int>(bits));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }
 
  return OK;
}


int64_t StreamPeerSerial::get_data_bits() const {
  sp_port_config *config = nullptr;
  int64_t data = 0;

  if(serial != nullptr) {
    if(auto err = sp_new_config(&config); err < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if((err = sp_get_config(serial, config)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int bits = 0; (err = sp_get_config_bits(config, &bits)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else {
      data = bits;
    }

    if(config != nullptr) {
      sp_free_config(config);
      config = nullptr;
    }
  }

  return data;
}


Error StreamPeerSerial::set_parity(SerialPortConfig::Parity parity) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_parity(serial, static_cast<sp_parity>(parity));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }
 
  return OK;
}


SerialPortConfig::Parity StreamPeerSerial::get_parity() const {
  sp_port_config *config = nullptr;
  SerialPortConfig::Parity parity = SerialPortConfig::INVALID_PARITY;

  if(serial != nullptr) {
    if(auto err = sp_new_config(&config); err < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if((err = sp_get_config(serial, config)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(sp_parity par = SP_PARITY_INVALID; (err = sp_get_config_parity(config, &par)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else {
      parity = static_cast<SerialPortConfig::Parity>(par);
    }

    if(config != nullptr) {
      sp_free_config(config);
      config = nullptr;
    }
  }

  return parity;
}


Error StreamPeerSerial::set_stop_bits(int64_t bits) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_stopbits(serial, static_cast<int>(bits));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }
 
  return OK;
}


int64_t StreamPeerSerial::get_stop_bits() const {
  sp_port_config *config = nullptr;
  int64_t data = 0;

  if(serial != nullptr) {
    if(auto err = sp_new_config(&config); err < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if((err = sp_get_config(serial, config)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int bits = 0; (err = sp_get_config_stopbits(config, &bits)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else {
      data = bits;
    }

    if(config != nullptr) {
      sp_free_config(config);
      config = nullptr;
    }
  }

  return data;
}


Error StreamPeerSerial::set_flow_control(SerialPortConfig::FlowControl flow) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_flowcontrol(serial, static_cast<sp_flowcontrol>(flow));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }
 
  return OK;
}


Error StreamPeerSerial::flush() {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  if(auto err = sp_drain(serial); err < 0) {
    _process_error(__FUNCTION__, err);
    return ERR_FILE_CANT_WRITE;
  }

  return OK;
}


Error StreamPeerSerial::apply_config(const Ref<SerialPortConfig> &config) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  if(config.is_null()) {
    return ERR_INVALID_PARAMETER;
  }

  if(auto err = sp_set_config(serial, config->ptr()); err < 0) {
    _process_error(__FUNCTION__, err);
    return ERR_FILE_CANT_WRITE;
  }

  return OK;
}


Ref<SerialPortConfig> StreamPeerSerial::get_config() {
  Ref<SerialPortConfig> config;

  if(serial != nullptr) {
    config.instantiate();
    if(auto err = sp_get_config(serial, config->ptr()); err != SP_OK) {
      _process_error(__FUNCTION__, err);
      config.unref();
    }
  }

  return config;
}


Error StreamPeerSerial::set_rts(SerialPortConfig::Rts level) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_rts(serial, static_cast<sp_rts>(level));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_cts(SerialPortConfig::Cts level) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_cts(serial, static_cast<sp_cts>(level));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_dtr(SerialPortConfig::Dtr level) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_dtr(serial, static_cast<sp_dtr>(level));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_dsr(SerialPortConfig::Dsr level) {
  if(serial == nullptr) {
    return ERR_DOES_NOT_EXIST;
  }

  auto err = sp_set_dsr(serial, static_cast<sp_dsr>(level));
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return FAILED;
  }

  return OK;
}


bool StreamPeerSerial::get_cts() {
  if(serial == nullptr) {
    return false;
  }

  sp_signal mask = static_cast<sp_signal>(0);
  auto err = sp_get_signals(serial, &mask);
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return false;
  }

  return (mask & SP_SIG_CTS) != 0;
}


bool StreamPeerSerial::get_dsr() {
  if(serial == nullptr) {
    return false;
  }

  sp_signal mask = static_cast<sp_signal>(0);
  auto err = sp_get_signals(serial, &mask);
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return false;
  }

  return (mask & SP_SIG_DSR) != 0;
}


bool StreamPeerSerial::get_ri() {
  if(serial == nullptr) {
    return false;
  }

  sp_signal mask = static_cast<sp_signal>(0);
  auto err = sp_get_signals(serial, &mask);
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return false;
  }

  return (mask & SP_SIG_RI) != 0;
}


bool StreamPeerSerial::get_cd() {
  if(serial == nullptr) {
    return false;
  }

  sp_signal mask = static_cast<sp_signal>(0);
  auto err = sp_get_signals(serial, &mask);
  if(err != SP_OK) {
    _process_error(__FUNCTION__, err);
    return false;
  }

  return (mask & SP_SIG_DCD) != 0;
}


String StreamPeerSerial::_to_string() const {
  Dictionary ser_info;
  ser_info["port"] = get_port();
  if(serial != nullptr) {
    sp_port_config *config = nullptr;
    if(auto err = sp_new_config(&config); err < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if((err = sp_get_config(serial, config)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int rate = 0; (err = sp_get_config_baudrate(config, &rate)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int data = 0; (err = sp_get_config_bits(config, &data)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(sp_parity par = SP_PARITY_INVALID; (err = sp_get_config_parity(config, &par)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else if(int stop = 0; (err = sp_get_config_stopbits(config, &stop)) < 0) {
      _process_error(__FUNCTION__, err);
    }
    else {
      ser_info["baudrate"]  = rate;
      ser_info["data_bits"] = data;
      ser_info["parity"]    = par;
      ser_info["stop_bits"] = stop;
    }

    if(config != nullptr) {
      sp_free_config(config);
      config = nullptr;
    }
  }

  return String("[StreamPeerSerial: {_}]").format(ser_info);
}


void StreamPeerSerial::_bind_methods() {
  ClassDB::bind_static_method("StreamPeerSerial", D_METHOD("list_ports"),                  &StreamPeerSerial::list_ports);
  ClassDB::bind_static_method("StreamPeerSerial", D_METHOD("open_port", "port", "config"), &StreamPeerSerial::open_port);

  ClassDB::bind_method(D_METHOD("_data_received", "data"),          &StreamPeerSerial::_data_received);
  ClassDB::bind_method(D_METHOD("_on_error",      "where", "what"), &StreamPeerSerial::_on_error);

  ClassDB::bind_method(D_METHOD("start_monitoring", "msec"), &StreamPeerSerial::start_monitoring, DEFVAL(50));
  ClassDB::bind_method(D_METHOD("stop_monitoring"),          &StreamPeerSerial::stop_monitoring);

  ClassDB::bind_method(D_METHOD("open", "port"), &StreamPeerSerial::open);
  ClassDB::bind_method(D_METHOD("is_open"),      &StreamPeerSerial::is_open);
  ClassDB::bind_method(D_METHOD("close"),        &StreamPeerSerial::close);

  ClassDB::bind_method(D_METHOD("wait_readable"),     &StreamPeerSerial::wait_readable);
  ClassDB::bind_method(D_METHOD("wait_writable"),     &StreamPeerSerial::wait_writable);
  ClassDB::bind_method(D_METHOD("read_raw", "size"),  &StreamPeerSerial::read_raw, DEFVAL(1));
  ClassDB::bind_method(D_METHOD("write_raw", "data"), &StreamPeerSerial::write_raw);

  ClassDB::bind_method(D_METHOD("get_port"),                   &StreamPeerSerial::get_port);
  ClassDB::bind_method(D_METHOD("get_config"),                 &StreamPeerSerial::get_config);
  ClassDB::bind_method(D_METHOD("set_config",       "config"), &StreamPeerSerial::apply_config);
  ClassDB::bind_method(D_METHOD("set_baudrate",     "rate"),   &StreamPeerSerial::set_baudrate);
  ClassDB::bind_method(D_METHOD("get_baudrate"),               &StreamPeerSerial::get_baudrate);
  ClassDB::bind_method(D_METHOD("set_data_bits",    "bits"),   &StreamPeerSerial::set_data_bits);
  ClassDB::bind_method(D_METHOD("get_data_bits"),              &StreamPeerSerial::get_data_bits);
  ClassDB::bind_method(D_METHOD("set_parity",       "parity"), &StreamPeerSerial::set_parity);
  ClassDB::bind_method(D_METHOD("get_parity"),                 &StreamPeerSerial::get_parity);
  ClassDB::bind_method(D_METHOD("set_stop_bits",    "bits"),   &StreamPeerSerial::set_stop_bits);
  ClassDB::bind_method(D_METHOD("get_stop_bits"),              &StreamPeerSerial::get_stop_bits);
  ClassDB::bind_method(D_METHOD("set_flow_control", "flow"),   &StreamPeerSerial::set_flow_control);

  ClassDB::bind_method(D_METHOD("flush"),            &StreamPeerSerial::flush);
  ClassDB::bind_method(D_METHOD("set_rts", "level"), &StreamPeerSerial::set_rts, DEFVAL(SerialPortConfig::RTS_ON));
  ClassDB::bind_method(D_METHOD("set_cts", "level"), &StreamPeerSerial::set_rts, DEFVAL(SerialPortConfig::CTS_FLOW));
  ClassDB::bind_method(D_METHOD("set_dtr", "level"), &StreamPeerSerial::set_dtr, DEFVAL(SerialPortConfig::DTR_ON));
  ClassDB::bind_method(D_METHOD("set_dsr", "level"), &StreamPeerSerial::set_dtr, DEFVAL(SerialPortConfig::DSR_FLOW));
  ClassDB::bind_method(D_METHOD("get_cts"),          &StreamPeerSerial::get_cts);
  ClassDB::bind_method(D_METHOD("get_dsr"),          &StreamPeerSerial::get_dsr);
  ClassDB::bind_method(D_METHOD("get_ri"),           &StreamPeerSerial::get_ri);
  ClassDB::bind_method(D_METHOD("get_cd"),           &StreamPeerSerial::get_cd);

  ADD_SIGNAL(MethodInfo("got_error",     PropertyInfo(Variant::STRING,            "where"), PropertyInfo(Variant::STRING, "what")));
  ADD_SIGNAL(MethodInfo("opened",        PropertyInfo(Variant::STRING,            "port")));
  ADD_SIGNAL(MethodInfo("data_received", PropertyInfo(Variant::PACKED_BYTE_ARRAY, "data")));
  ADD_SIGNAL(MethodInfo("closed",        PropertyInfo(Variant::STRING,            "port")));
}

