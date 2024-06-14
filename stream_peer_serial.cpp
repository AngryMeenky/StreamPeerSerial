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

using namespace godot;
#else
#include "core/object/class_db.h"
#include "core/os/memory.h"
#include "core/os/os.h"
#endif
#include <string>

using namespace std::chrono;

void StreamPeerSerial::_data_received(const PackedByteArray &buf) {
  emit_signal("data_received", buf);
}


StreamPeerSerial::StreamPeerSerial(const String &port, uint32_t baudrate, uint32_t timeout, ByteSize bytesize, Parity parity, StopBits stopbits, FlowControl flowcontrol) {
  serialerror_t err = serialerror_success;
  serial = new Serial(port.ascii().get_data(),
      baudrate, Timeout::simpleTimeout(timeout), bytesize_t(bytesize),
      parity_t(parity), stopbits_t(stopbits), flowcontrol_t(flowcontrol), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }
}


StreamPeerSerial::~StreamPeerSerial() {
  close();
  stop_monitoring();
  delete serial;
}


Dictionary StreamPeerSerial::list_ports() {
  std::vector<PortInfo> ports_info = serial::list_ports();

  Dictionary info_dict;
  for (PortInfo port : ports_info) {
    Dictionary info;
    info["desc"] = port.description.c_str();
    info["hw_id"] = port.hardware_id.c_str();
    info_dict[port.port.c_str()] = Variant(info);
  }

  return info_dict;
}


Ref<StreamPeerSerial> StreamPeerSerial::open_port(
  const String   &port, uint32_t baud,   uint32_t timeout,
        ByteSize  size, Parity   parity, StopBits stop,    FlowControl flow) {
  Ref<StreamPeerSerial> srs;

  // only attempt to open the port if the string isn't empty
  if(!port.is_empty()) {
    srs.instantiate();

    // use goto without using goto
    while(srs.is_valid()) {
      serialerror_t err = serialerror_success;
      srs->serial->setBaudrate(baud, &err);
      if(err != serialerror_success) {
        srs.unref();
        break;
      }

      // setting the timeout can't fail
      srs->serial->setTimeout(Timeout::max(), timeout, 0, timeout, 0);

      srs->serial->setBytesize(bytesize_t(size), &err);
      if(err != serialerror_success) {
        srs.unref();
        break;
      }

      srs->serial->setParity(parity_t(parity), &err);
      if(err != serialerror_success) {
        srs.unref();
        break;
      }

      srs->serial->setStopbits(stopbits_t(stop), &err);
      if(err != serialerror_success) {
        srs.unref();
        break;
      }

      srs->serial->setFlowcontrol(flowcontrol_t(flow), &err);
      if(err != serialerror_success) {
        srs.unref();
        break;
      }

      // attempt to open the serial device
      srs->serial->setPort(port.ascii().get_data(), &err);
      if(err != serialerror_success || srs->serial->open() != serialerror_success) {
        srs.unref();
        break;
      }

      break;
    }
  }

  return srs;
}


void StreamPeerSerial::_on_error(const String &where, const String &what) {
  fine_working = false;
  error_message = "[" + get_port() + "] Error at " + where + ": " + what;
  // ERR_FAIL_MSG(error_message);
  emit_signal("got_error", where, what);
}


Error StreamPeerSerial::start_monitoring(uint64_t interval_in_usec) {
  ERR_FAIL_COND_V_MSG(!monitoring_should_exit, ERR_ALREADY_IN_USE, "Monitor already started.");
  stop_monitoring();
  monitoring_should_exit = false;
  monitoring_interval = interval_in_usec;
  thread = std::thread(_thread_func, this);
  fine_working = is_open();

  return OK;
}


void StreamPeerSerial::stop_monitoring() {
  monitoring_should_exit = true;
  if(thread.joinable()) {
    thread.join();
  }
}


void StreamPeerSerial::_thread_func(void *p_user_data) {
  StreamPeerSerial *serial_port = static_cast<StreamPeerSerial *>(p_user_data);

  while(!serial_port->monitoring_should_exit) {
    time_point time_start = system_clock::now();

    if(serial_port->fine_working && serial_port->_get_available_bytes() > 0) {
       serial_port->call_deferred(
         "_data_received", serial_port->read_raw(serial_port->_get_available_bytes())
       );
    }

    time_t time_elapsed = duration_cast<microseconds>(system_clock::now() - time_start).count();
    if(time_elapsed < serial_port->monitoring_interval) {
      std::this_thread::sleep_for(microseconds(serial_port->monitoring_interval - time_elapsed));
    }
  }
}


Error StreamPeerSerial::open(String port) {
  error_message = "";
  if(serial->isOpen()) {
    close();
  }
 
  if(!port.is_empty() && set_port(port) != OK) {
    return FAILED;
  }

  if(auto err = serial->open(); err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    switch(err) {
      case serialerror_io_failed:
        return ERR_CANT_OPEN;
      break;
      case serialerror_serial:
        return ERR_ALREADY_IN_USE;
      break;
      case serialerror_argument:
        return ERR_INVALID_PARAMETER;
      break;
      default:
        return FAILED;
      break;
    }
  }

  fine_working = true;
  emit_signal("opened", port);
  return OK;
}


bool StreamPeerSerial::is_open() const {
  return serial->isOpen();
}


void StreamPeerSerial::close() {
  if(serial->close() != serialerror_success) {
    _on_error(__FUNCTION__, serial->getLastError().c_str());
  }

  fine_working = false;
  emit_signal("closed", serial->getPort().c_str());
}


int32_t StreamPeerSerial::_get_available_bytes() const {
  serialerror_t err = serialerror_success;
  auto retVal = serial->available(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }

  return retVal;
}


bool StreamPeerSerial::wait_readable() {
  serialerror_t err = serialerror_success;
  auto retVal = serial->waitReadable(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }
 
  return retVal;
}


void StreamPeerSerial::wait_byte_times(size_t count) {
  serial->waitByteTimes(count);
}


PackedByteArray StreamPeerSerial::read_raw(size_t size) {
  PackedByteArray raw;
  std::vector<uint8_t> buf_temp;
  serialerror_t err = serialerror_success;
  size_t bytes_read = serial->read(buf_temp, size, &err);

  if (bytes_read > 0 && raw.resize(bytes_read) == OK) {
    memcpy(raw.ptrw(), (const char *)buf_temp.data(), bytes_read);
    call_deferred("_on_error", String(__FUNCTION__), String("Unable to allocate space"));
  }

  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }
 
  return raw;
}


Error StreamPeerSerial::_get_data(uint8_t *p_buffer, int32_t bytes, int32_t *r_received) {
   if(bytes < 0) {
     return ERR_PARAMETER_RANGE_ERROR;
   }

   Error result = OK;
   serialerror_t err = serialerror_success;
   if((*r_received = serial->read(p_buffer, bytes, &err)) == 0) {
     result = ERR_BUSY;
   }

   if(err != serialerror_success) {
     call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
     switch(err) {
       case serialerror_not_opened:
         result = ERR_FILE_CANT_OPEN;
       break;
       case serialerror_io_failed:
         result = ERR_FILE_CANT_READ;
       break;
       case serialerror_serial:
         result = ERR_FILE_CORRUPT;
       break;
       default:
         result = ERR_UNAVAILABLE;
       break;
     }
  }

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
   serialerror_t err = serialerror_success;
   auto retVal = serial->write(data.ptr(), data.size(), &err);
   if(err != serialerror_success) {
     call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
   }
 
   return retVal;
}


Error StreamPeerSerial::_put_data(const uint8_t *p_data, int32_t bytes, int32_t *r_sent) {
  if(bytes < 0) {
    return ERR_PARAMETER_RANGE_ERROR;
  }

  Error result = OK;
  serialerror_t err = serialerror_success;
  do {
    if(int sent = static_cast<int>(serial->write(p_data, bytes, &err)); sent == 0) {
       std::this_thread::sleep_for(milliseconds(serial->getTimeout().write_timeout_constant));
    }
    else {
      *r_sent += sent;
      p_data += sent;
      bytes -= sent;
    }
  } while(err == serialerror_success && bytes > 0);
 
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    switch(err) {
      case serialerror_not_opened:
        result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_io_failed:
        result = ERR_FILE_CANT_READ;
      break;
      case serialerror_serial:
        result = ERR_FILE_CORRUPT;
      break;
      default:
        result = ERR_UNAVAILABLE;
      break;
    }
  }

  return result;
}


Error StreamPeerSerial::_put_partial_data(const uint8_t *p_data, int32_t bytes, int32_t *r_sent) {
  Error result = OK;
  serialerror_t err = serialerror_success;
  if((*r_sent = static_cast<int>(serial->write(p_data, bytes, &err))) == 0) {
    result = ERR_BUSY;
  }
 
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    switch(err) {
      case serialerror_not_opened:
        result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_io_failed:
        result = ERR_FILE_CANT_READ;
      break;
      case serialerror_serial:
        result = ERR_FILE_CORRUPT;
      break;
      default:
        result = ERR_UNAVAILABLE;
      break;
    }
  }

  return result;
}


String StreamPeerSerial::read_line(size_t max_length, String eol, bool utf8_encoding) {
  serialerror_t err = serialerror_success;
  auto line = serial->readline(max_length, eol.utf8().get_data(), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }
 
  return "";
  if(utf8_encoding) {
    String str;
    str.parse_utf8(line.c_str());
    return str;
  }
 
  return line.c_str();
}


PackedStringArray StreamPeerSerial::read_lines(size_t max_length, String eol, bool utf8_encoding) {
  PackedStringArray lines;
  serialerror_t err = serialerror_success;
  if(utf8_encoding) {
    for(std::string line : serial->readlines(max_length, eol.utf8().get_data())) {
      String str;
      str.parse_utf8(line.c_str());
      lines.append(str);
    }
  }
  else {
    for(std::string line : serial->readlines(max_length, eol.utf8().get_data())) {
      lines.append(line.c_str());
    }
  }
 
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }
 
  return lines;
}


Error StreamPeerSerial::set_port(const String &port) {
  Error result = OK;
  serialerror_t err = serialerror_success;
  serial->setPort(port.ascii().get_data(), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    switch(err) {
      case serialerror_argument:
        result = ERR_INVALID_PARAMETER;
      break;
      case serialerror_io_failed:
        result = ERR_FILE_CANT_OPEN;
      break;
      case serialerror_serial:
        result = ERR_ALREADY_IN_USE;
      break;
      default:
        result = FAILED;
      break;
    }
  }
 
  return result;
}


String StreamPeerSerial::get_port() const {
  serialerror_t err = serialerror_success;
  auto port = serial->getPort(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return port.c_str();
}


Error StreamPeerSerial::set_timeout(uint32_t timeout) {
  serial->setTimeout(Timeout::max(), timeout, 0, timeout, 0);
  return OK;
}


uint32_t StreamPeerSerial::get_timeout() const {
  return serial->getTimeout().read_timeout_constant;
}


Error StreamPeerSerial::set_baudrate(uint32_t baudrate) {
   serialerror_t err = serialerror_success;
   serial->setBaudrate(baudrate, &err);
   if(err != serialerror_success) {
     call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
     return FAILED;
   }
 
   return OK;
}


uint32_t StreamPeerSerial::get_baudrate() const {
  serialerror_t err = serialerror_success;
  auto baud = serial->getBaudrate(&err);
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return baud;
}


Error StreamPeerSerial::set_bytesize(ByteSize bytesize) {
  serialerror_t err = serialerror_success;
  serial->setBytesize(bytesize_t(bytesize), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }
 
  return OK;
}


StreamPeerSerial::ByteSize StreamPeerSerial::get_bytesize() const {
  serialerror_t err = serialerror_success;
  auto sz = ByteSize(serial->getBytesize(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return sz;
}


Error StreamPeerSerial::set_parity(Parity parity) {
  serialerror_t err = serialerror_success;
  serial->setParity(parity_t(parity), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


StreamPeerSerial::Parity StreamPeerSerial::get_parity() const {
  serialerror_t err = serialerror_success;
  auto par = Parity(serial->getParity(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return par;
}


Error StreamPeerSerial::set_stopbits(StopBits stopbits) {
  serialerror_t err = serialerror_success;
  serial->setStopbits(stopbits_t(stopbits), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }
 
  return OK;
}


StreamPeerSerial::StopBits StreamPeerSerial::get_stopbits() const {
  serialerror_t err = serialerror_success;
  auto bits = StopBits(serial->getStopbits(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return bits;
}


Error StreamPeerSerial::set_flowcontrol(FlowControl flowcontrol) {
  serialerror_t err = serialerror_success;
  serial->setFlowcontrol(flowcontrol_t(flowcontrol), &err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }
 
  return OK;
}


StreamPeerSerial::FlowControl StreamPeerSerial::get_flowcontrol() const {
  serialerror_t err = serialerror_success;
  auto flow = FlowControl(serial->getFlowcontrol(&err));
  if(err != serialerror_success) {
    const_cast<StreamPeerSerial *>(this)->call_deferred(
       "_on_error", String(__FUNCTION__), String(serial->getLastError().c_str())
    );
  }
 
  return flow;
}


Error StreamPeerSerial::flush() {
  if(serial->flush() != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::flush_input() {
  if(serial->flushInput() != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::flush_output() {
  if(serial->flushOutput() != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::send_break(int duration) {
  if(serial->sendBreak(duration) != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_break(bool level) {
  if(serial->setBreak(level) != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_rts(bool level) {
  if(serial->setRTS(level) != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


Error StreamPeerSerial::set_dtr(bool level) {
  if(serial->setDTR(level) != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
    return FAILED;
  }

  return OK;
}


bool StreamPeerSerial::wait_for_change() {
  serialerror_t err = serialerror_success;
  auto result = serial->waitForChange(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }

  return result;
}


bool StreamPeerSerial::get_cts() {
  serialerror_t err = serialerror_success;
  auto result = serial->getCTS(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }

  return result;
}


bool StreamPeerSerial::get_dsr() {
  serialerror_t err = serialerror_success;
  auto result = serial->getDSR(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }

  return result;
}


bool StreamPeerSerial::get_ri() {
  serialerror_t err = serialerror_success;
  auto result = serial->getRI(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }

  return result;
}


bool StreamPeerSerial::get_cd() {
  serialerror_t err = serialerror_success;
  auto result = serial->getCD(&err);
  if(err != serialerror_success) {
    call_deferred("_on_error", String(__FUNCTION__), String(serial->getLastError().c_str()));
  }

  return result;
}


String StreamPeerSerial::_to_string() const {
  Dictionary ser_info;
  ser_info["port"] = get_port();
  ser_info["baudrate"] = get_baudrate();
  ser_info["byte_size"] = get_bytesize();
  ser_info["parity"] = get_parity();
  ser_info["stop_bits"] = get_stopbits();

  return String("[StreamPeerSerial: {_}]").format(ser_info);
}


void StreamPeerSerial::_bind_methods() {
  ClassDB::bind_static_method("StreamPeerSerial", D_METHOD("list_ports"), &StreamPeerSerial::list_ports);
  ClassDB::bind_static_method("StreamPeerSerial", D_METHOD("open_port", "port", "baudrate", "timeout", "bytesize", "parity", "stopbits", "flowcontrol"), &StreamPeerSerial::open_port, DEFVAL(9600U), DEFVAL(0U), DEFVAL(BYTESIZE_8), DEFVAL(PARITY_NONE), DEFVAL(STOPBITS_1), DEFVAL(FLOWCONTROL_NONE));

  ClassDB::bind_method(D_METHOD("_data_received", "data"), &StreamPeerSerial::_data_received);
  ClassDB::bind_method(D_METHOD("_on_error", "where", "what"), &StreamPeerSerial::_on_error);
  ClassDB::bind_method(D_METHOD("is_in_error"), &StreamPeerSerial::is_in_error);
  ClassDB::bind_method(D_METHOD("get_last_error"), &StreamPeerSerial::get_last_error);

  ClassDB::bind_method(D_METHOD("start_monitoring", "interval_in_usec"), &StreamPeerSerial::start_monitoring, DEFVAL(10000));
  ClassDB::bind_method(D_METHOD("stop_monitoring"), &StreamPeerSerial::stop_monitoring);

  ClassDB::bind_method(D_METHOD("open", "port"), &StreamPeerSerial::open, DEFVAL(""));
  ClassDB::bind_method(D_METHOD("is_open"), &StreamPeerSerial::is_open);
  ClassDB::bind_method(D_METHOD("close"), &StreamPeerSerial::close);

  ClassDB::bind_method(D_METHOD("wait_readable"), &StreamPeerSerial::wait_readable);
  ClassDB::bind_method(D_METHOD("wait_byte_times", "count"), &StreamPeerSerial::wait_byte_times);
  ClassDB::bind_method(D_METHOD("read_raw", "size"), &StreamPeerSerial::read_raw, DEFVAL(1));
  ClassDB::bind_method(D_METHOD("write_raw", "data"), &StreamPeerSerial::write_raw);
  ClassDB::bind_method(D_METHOD("read_line", "max_len", "eol", "utf8_encoding"), &StreamPeerSerial::read_line, DEFVAL(65535), DEFVAL("\n"), DEFVAL(false));
  ClassDB::bind_method(D_METHOD("read_lines", "max_len", "eol", "utf8_encoding"), &StreamPeerSerial::read_lines, DEFVAL(65535), DEFVAL("\n"), DEFVAL(false));

  ClassDB::bind_method(D_METHOD("set_port", "port"), &StreamPeerSerial::set_port);
  ClassDB::bind_method(D_METHOD("get_port"), &StreamPeerSerial::get_port);
  ClassDB::bind_method(D_METHOD("set_baudrate", "baudrate"), &StreamPeerSerial::set_baudrate);
  ClassDB::bind_method(D_METHOD("get_baudrate"), &StreamPeerSerial::get_baudrate);
  ClassDB::bind_method(D_METHOD("set_timeout", "timeout"), &StreamPeerSerial::set_timeout);
  ClassDB::bind_method(D_METHOD("get_timeout"), &StreamPeerSerial::get_timeout);
  ClassDB::bind_method(D_METHOD("set_bytesize", "bytesize"), &StreamPeerSerial::set_bytesize);
  ClassDB::bind_method(D_METHOD("get_bytesize"), &StreamPeerSerial::get_bytesize);
  ClassDB::bind_method(D_METHOD("set_parity", "parity"), &StreamPeerSerial::set_parity);
  ClassDB::bind_method(D_METHOD("get_parity"), &StreamPeerSerial::get_parity);
  ClassDB::bind_method(D_METHOD("set_stopbits", "stopbits"), &StreamPeerSerial::set_stopbits);
  ClassDB::bind_method(D_METHOD("get_stopbits"), &StreamPeerSerial::get_stopbits);
  ClassDB::bind_method(D_METHOD("set_flowcontrol", "flowcontrol"), &StreamPeerSerial::set_flowcontrol);
  ClassDB::bind_method(D_METHOD("get_flowcontrol"), &StreamPeerSerial::get_flowcontrol);

  ClassDB::bind_method(D_METHOD("flush"), &StreamPeerSerial::flush);
  ClassDB::bind_method(D_METHOD("flush_input"), &StreamPeerSerial::flush_input);
  ClassDB::bind_method(D_METHOD("flush_output"), &StreamPeerSerial::flush_output);
  ClassDB::bind_method(D_METHOD("send_break", "duration"), &StreamPeerSerial::send_break);
  ClassDB::bind_method(D_METHOD("set_break", "level"), &StreamPeerSerial::set_break, DEFVAL(true));
  ClassDB::bind_method(D_METHOD("set_rts", "level"), &StreamPeerSerial::set_rts, DEFVAL(true));
  ClassDB::bind_method(D_METHOD("set_dtr", "level"), &StreamPeerSerial::set_dtr, DEFVAL(true));
  ClassDB::bind_method(D_METHOD("wait_for_change"), &StreamPeerSerial::wait_for_change);
  ClassDB::bind_method(D_METHOD("get_cts"), &StreamPeerSerial::get_cts);
  ClassDB::bind_method(D_METHOD("get_dsr"), &StreamPeerSerial::get_dsr);
  ClassDB::bind_method(D_METHOD("get_ri"), &StreamPeerSerial::get_ri);
  ClassDB::bind_method(D_METHOD("get_cd"), &StreamPeerSerial::get_cd);

  ADD_PROPERTY(PropertyInfo(Variant::STRING, "port"), "set_port", "get_port");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "baudrate"), "set_baudrate", "get_baudrate");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "timeout"), "set_timeout", "get_timeout");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "bytesize", PROPERTY_HINT_ENUM, "5, 6, 7, 8"), "set_bytesize", "get_bytesize");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "parity", PROPERTY_HINT_ENUM, "None, Odd, Even, Mark, Space"), "set_parity", "get_parity");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "stopbits", PROPERTY_HINT_ENUM, "1, 2, 1.5"), "set_stopbits", "get_stopbits");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "flowcontrol", PROPERTY_HINT_ENUM, "None, Software, Hardware"), "set_flowcontrol", "get_flowcontrol");

#ifndef GDEXTENSION
  ADD_PROPERTY_DEFAULT("port", "");
  ADD_PROPERTY_DEFAULT("baudrate", 9600);
  ADD_PROPERTY_DEFAULT("timeout", 0);
  ADD_PROPERTY_DEFAULT("bytesize", BYTESIZE_8);
  ADD_PROPERTY_DEFAULT("parity", PARITY_NONE);
  ADD_PROPERTY_DEFAULT("stopbits", STOPBITS_1);
  ADD_PROPERTY_DEFAULT("flowcontrol", FLOWCONTROL_NONE);
#endif

  ADD_SIGNAL(MethodInfo("got_error", PropertyInfo(Variant::STRING, "where"), PropertyInfo(Variant::STRING, "what")));
  ADD_SIGNAL(MethodInfo("opened", PropertyInfo(Variant::STRING, "port")));
  ADD_SIGNAL(MethodInfo("data_received", PropertyInfo(Variant::PACKED_BYTE_ARRAY, "data")));
  ADD_SIGNAL(MethodInfo("closed", PropertyInfo(Variant::STRING, "port")));

  BIND_ENUM_CONSTANT(BYTESIZE_5);
  BIND_ENUM_CONSTANT(BYTESIZE_6);
  BIND_ENUM_CONSTANT(BYTESIZE_7);
  BIND_ENUM_CONSTANT(BYTESIZE_8);

  BIND_ENUM_CONSTANT(PARITY_NONE);
  BIND_ENUM_CONSTANT(PARITY_ODD);
  BIND_ENUM_CONSTANT(PARITY_EVEN);
  BIND_ENUM_CONSTANT(PARITY_MARK);
  BIND_ENUM_CONSTANT(PARITY_SPACE);

  BIND_ENUM_CONSTANT(STOPBITS_1);
  BIND_ENUM_CONSTANT(STOPBITS_2);
  BIND_ENUM_CONSTANT(STOPBITS_1P5);

  BIND_ENUM_CONSTANT(FLOWCONTROL_NONE);
  BIND_ENUM_CONSTANT(FLOWCONTROL_SOFTWARE);
  BIND_ENUM_CONSTANT(FLOWCONTROL_HARDWARE);
}

